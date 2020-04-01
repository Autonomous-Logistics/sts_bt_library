/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*  Modified work Copyright (C) 2019-2020 Sebastian Mueller, Nick Fiege, Robert Rudolph
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <sts_bt_library/behavior_tree.h>


#include <sts_bt_library/dot_bt.h>
#include <sts_bt_library/graphml/parser.h>
#include <sts_bt_library/sts_bt_if_node.h>

#include <sts_bt_library/Event.h>
#include <sts_bt_library/bt_event.h>


BT::DotBt* sts_behavior_tree::BehaviorTree::dotBtPtr = NULL;
BT::Logger* sts_behavior_tree::BehaviorTree::loggerPtr = NULL;

ros::NodeHandle* sts_behavior_tree::BehaviorTree::lastNhPtr = NULL;
BT::TreeNode* sts_behavior_tree::BehaviorTree::lastRootPtr = NULL;
std::string sts_behavior_tree::BehaviorTree::lastGraphmlPath;
int sts_behavior_tree::BehaviorTree::lastTickPeriod = 0;
unsigned int sts_behavior_tree::BehaviorTree::logLevel = 0;
ros::Publisher sts_behavior_tree::BehaviorTree::eventStrPub_;
ros::Publisher sts_behavior_tree::BehaviorTree::eventPub_;


std::mutex sts_behavior_tree::BehaviorTree::dataAccessMutex_;

void sts_behavior_tree::BehaviorTree::setLogLevel(int log_level)
{
    sts_behavior_tree::BehaviorTree::logLevel = log_level;
}

void sts_behavior_tree::BehaviorTree::handleEvent(BT::Event e)
{
    /// [fgn] do we really dont want to buffer those events?
    static std_msgs::String msg;
    /// we could do
    //#include "boost/date_time/posix_time/posix_time.hpp"
    //boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost());
    msg.data = e.toString();
    sts_behavior_tree::BehaviorTree::eventStrPub_.publish(msg);
    sts_behavior_tree::BehaviorTree::eventPub_.publish(e.toRosMsg());
}

void sts_behavior_tree::BehaviorTree::statusChangedEvent(BT::TreeNode* node, BT::ReturnStatus status)
{
    if (sts_behavior_tree::BehaviorTree::dotBtPtr != NULL)
        sts_behavior_tree::BehaviorTree::dotBtPtr->statusChangedEvent(node, status);
}

void sts_behavior_tree::BehaviorTree::Execute(BT::TreeNode* root, int TickPeriod_milliseconds, std::string& id, ros::NodeHandle* nh, std::string nodeName, bool events, bool feedback, int debug_root_delay_)
{
    if (TickPeriod_milliseconds == 0)
    {
        ros::shutdown();
    }

    //Setup reload callback
    ros::ServiceServer serviceReload_ = nh->advertiseService("/sts_bt_visualisation_node/reloadGraphMl", ReLoad);

    ros::Publisher pub;
    std_msgs::String message;
    if(feedback)
        pub = nh->advertise<std_msgs::String>("feedback", 1);
    if(events)
    {
        sts_behavior_tree::BehaviorTree::eventStrPub_ = nh->advertise<std_msgs::String>("events_str", 1);
        sts_behavior_tree::BehaviorTree::eventPub_ = nh->advertise<sts_bt_library::bt_event>("events", 1);

    }
    init(root, TickPeriod_milliseconds, nh);

    while (ros::ok())
    {
        //Cycle time should be deterministic. Only wait the remaining time after execution.
        auto timePointStart = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> guard(sts_behavior_tree::BehaviorTree::dataAccessMutex_);
            syncInputBuffers(sts_behavior_tree::BehaviorTree::lastRootPtr);

            if(sts_behavior_tree::BehaviorTree::loggerPtr != NULL)
                if(loggerPtr->isPrintAllowed(2))
                    ROS_INFO("\n\n --------- Ticking root ---------\n");

            // Ticking the root node
            sts_behavior_tree::BehaviorTree::lastRootPtr->Tick(id);

            // Printing its state
            // root->GetNodeState();
            dotBtPtr->triggerPublish();

            //Sleep debug to be able  to see gui visualization
            std::this_thread::sleep_for(std::chrono::milliseconds(debug_root_delay_));

            if (sts_behavior_tree::BehaviorTree::lastRootPtr->get_status() != BT::RUNNING)
            {
                // when the root returns a status it resets the colors of the tree
                sts_behavior_tree::BehaviorTree::lastRootPtr->ResetColorState();
            }

            // flush logger (send and clear buffer)
            if (sts_behavior_tree::BehaviorTree::loggerPtr != NULL)
                sts_behavior_tree::BehaviorTree::loggerPtr->flush();
        }//end of scope lock

        if (feedback)
        {
            //Publish the bt feedback (last call in this cycle)
            message.data = id;
            pub.publish(message);
        }

        auto timePointEnd = std::chrono::steady_clock::now();
        auto executeDuration = std::chrono::duration_cast<std::chrono::milliseconds>(timePointEnd - timePointStart);

        if (executeDuration.count() >= TickPeriod_milliseconds)
        {
            //DEBUG_STDOUT("BT execution " << executeDuration.count() << " exceeded cycletime : " << TickPeriod_milliseconds);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds - executeDuration.count()));
        }

        ros::spinOnce();
        //auto timePointEnd2 = std::chrono::steady_clock::now();
        //auto executeDuration2 = std::chrono::duration_cast<std::chrono::milliseconds>(timePointEnd2 - timePointStart);
        //std::cout << "Ticking loop duration: " << executeDuration2.count() << std::endl;
    }

    if (sts_behavior_tree::BehaviorTree::dotBtPtr != NULL)
        delete sts_behavior_tree::BehaviorTree::dotBtPtr;
    if (sts_behavior_tree::BehaviorTree::loggerPtr != NULL)
        delete sts_behavior_tree::BehaviorTree::loggerPtr;
}

void sts_behavior_tree::BehaviorTree::syncInputBuffers(BT::TreeNode* root)
{
    /// cycle though all nodes and every sts_bt_if_node will react with
    /// flushing their input buffers (set last received value)
    /// by shifting the input register bits of every condition
    root->Sync();
}

void sts_behavior_tree::BehaviorTree::init(BT::TreeNode* root, int TickPeriod_milliseconds, ros::NodeHandle* nh)
{
    sts_behavior_tree::BehaviorTree::lastTickPeriod = TickPeriod_milliseconds;

    //######## NEW VISUALISATION ############
    sts_behavior_tree::BehaviorTree::dotBtPtr = new BT::DotBt(nh, root, "/sts_bt_visualisation_node/treeStructure", "/sts_bt_visualisation_node/treeUpdate", TickPeriod_milliseconds);
    std::thread t(&BT::DotBt::publish, dotBtPtr);
    t.detach();
    root->ResetColorState();

    //######## NEW LOGGER IF THERE IS NO LOGGER SET ############
    if(sts_behavior_tree::BehaviorTree::loggerPtr == NULL)
        sts_behavior_tree::BehaviorTree::loggerPtr = new BT::Logger(nh, sts_behavior_tree::BehaviorTree::logLevel);
    //################################
}

bool sts_behavior_tree::BehaviorTree::ReLoad(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::lock_guard<std::mutex> guard(sts_behavior_tree::BehaviorTree::dataAccessMutex_);

    delete sts_behavior_tree::BehaviorTree::lastRootPtr;
    dotBtPtr->triggerDestruction();
    delete sts_behavior_tree::BehaviorTree::dotBtPtr;

    Load(lastGraphmlPath, sts_behavior_tree::BehaviorTree::lastNhPtr);

    init(sts_behavior_tree::BehaviorTree::lastRootPtr, sts_behavior_tree::BehaviorTree::lastTickPeriod, sts_behavior_tree::BehaviorTree::lastNhPtr);

    res.success = true;

    return true;
}

BT::TreeNode* sts_behavior_tree::BehaviorTree::Load(std::string graphmlPath, ros::NodeHandle* nh)
{
    //######## NEW LOGGER ############
    sts_behavior_tree::BehaviorTree::loggerPtr = new BT::Logger(nh, sts_behavior_tree::BehaviorTree::logLevel);
    //################################

    graphml::Config treeCfg = graphml::parse(graphmlPath);

    std::vector<BT::TreeNode*> nodes;
    BT::TreeNode* rootPtr = sts_behavior_tree::BehaviorTree::createBTNodes(treeCfg, nodes, nh);

    sts_behavior_tree::BehaviorTree::connectBTNodes(treeCfg, nodes);

    //Save load configuration to be able to reload
    lastGraphmlPath = graphmlPath;
    sts_behavior_tree::BehaviorTree::lastNhPtr = nh;
    sts_behavior_tree::BehaviorTree::lastRootPtr = rootPtr;

    return rootPtr;
}

BT::TreeNode* sts_behavior_tree::BehaviorTree::createBTNodes(graphml::Config &treeCfg, std::vector<BT::TreeNode*> &nodes,  ros::NodeHandle* nh)
{
    BT::TreeNode* rootPtr = NULL;
    BT::Logger* loggerArg = sts_behavior_tree::BehaviorTree::loggerPtr;
    for(auto node : treeCfg.nodeList)
    {
        //std::cout << node << std::endl;
        switch(node.type)
        {
        case graphml::BTNode::BT_TYPE::ROOT:
            rootPtr = new BT::SequenceNode(node.id, node.text, loggerArg);
            nodes.push_back(rootPtr); break;
        case graphml::BTNode::BT_TYPE::PARALLEL:
            nodes.push_back(new BT::ParallelNode(node.id, node.text, loggerArg)); break;
        case graphml::BTNode::BT_TYPE::SEQUENCE:
            nodes.push_back(new BT::SequenceNode(node.id, node.text, loggerArg)); break;
        case graphml::BTNode::BT_TYPE::SEQUENCE_MEMORY:
            nodes.push_back(new BT::SequenceNodeWithMemory(node.id, node.text, loggerArg)); break;
        case graphml::BTNode::BT_TYPE::FALLBACK:
            nodes.push_back(new BT::FallbackNode(node.id, node.text, loggerArg)); break;
        case graphml::BTNode::BT_TYPE::FALLBACK_MEMORY:
            nodes.push_back(new BT::FallbackNodeWithMemory(node.id, node.text, loggerArg)); break;
        case graphml::BTNode::BT_TYPE::DECORATOR:
            break;
        case graphml::BTNode::BT_TYPE::FUNCTION:
            switch(node.function)
            {
            case graphml::BTNode::BT_FUNCTION::FALSE:
                nodes.push_back(new BT::FalseNode(node.id, node.text, loggerArg)); break;
            case graphml::BTNode::BT_FUNCTION::SET:
                nodes.push_back(new BT::SetNode(node.id, node.text, loggerArg)); break;
            case graphml::BTNode::BT_FUNCTION::RESET:
                nodes.push_back(new BT::ResetNode(node.id, node.text, loggerArg)); break;
            case graphml::BTNode::BT_FUNCTION::FLIPFLOP:
                nodes.push_back(new BT::FlipFlopNode(node.id, node.text, loggerArg)); break;
            default: break;
            }break;
        case graphml::BTNode::BT_TYPE::CONDITION:
            switch(node.function)
            {
            case graphml::BTNode::BT_FUNCTION::TOPIC:
                nodes.push_back(sts_bt_if_factory::createNode(node.id, node.text, true, sts_bt_if_factory::TOPIC, nh, false, loggerArg));
                break;
            case graphml::BTNode::BT_FUNCTION::SERVICE:
                nodes.push_back(sts_bt_if_factory::createNode(node.id, node.text, true, sts_bt_if_factory::SERVICE, nh, false, loggerArg));
                break;
            case graphml::BTNode::BT_FUNCTION::HOOK:
                nodes.push_back(new BT::ConditionNodeHook(node.id, node.text, loggerArg)); break;
                break;
            default: break;
            }break;

        case graphml::BTNode::BT_TYPE::ACTION:
            switch(node.function)
            {
            case graphml::BTNode::BT_FUNCTION::ACTION:
                nodes.push_back(sts_bt_if_factory::createNode(node.id, node.text, false, sts_bt_if_factory::ACTION, nh, false, loggerArg));
                break;
            case graphml::BTNode::BT_FUNCTION::SERVICE:
                nodes.push_back(sts_bt_if_factory::createNode(node.id, node.text, false, sts_bt_if_factory::SERVICE, nh, false, loggerArg));
                break;
            default:
                break;
            }break;

        default:
            std::cout << "[BT!] UNKNOWN NODE <"<<node.id<<"> PARSED" << std::endl;
            std::cout << "-- " << node.text << std::endl;
            std::cout << "-- TYPE["<< node.getTypeString() <<"], FUNCTION["<< node.getFunctionString() <<"]" << std::endl;
            break;
        }
    }

    return rootPtr;
}

void sts_behavior_tree::BehaviorTree::connectBTNodes(graphml::Config &treeCfg, std::vector<BT::TreeNode*> &nodes)
{
    for(auto node : treeCfg.nodeList)
    {
        /// grab real node from the config one
        BT::TreeNode* btNode = sts_behavior_tree::BehaviorTree::findNodeById(node.id, nodes);
        if(btNode != NULL)
        {
            ///check every child node and attach them to real node
            for(auto child : node.children)
            {
                BT::TreeNode* btChild = sts_behavior_tree::BehaviorTree::findNodeById(child->id, nodes);
                if(btChild != NULL)
                    btNode->AddChild(btChild);
            }
            ///check every child node and attach hooks to them
            for(auto hook : node.hooks)
            {
                BT::TreeNode* btChild = sts_behavior_tree::BehaviorTree::findNodeById(hook->id, nodes);
                if(btChild != NULL)
                    btNode->AddHook(btChild);
            }
        }
    }

}

BT::TreeNode* sts_behavior_tree::BehaviorTree::findNodeById(std::string &id, std::vector<BT::TreeNode*> &nodes)
{
    for(auto btnode : nodes)
    {
        if(btnode->get_id().compare(id) == 0)
            return btnode;
    }
    return NULL;
}
