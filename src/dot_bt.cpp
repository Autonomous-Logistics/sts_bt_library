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

#include <sts_bt_library/dot_bt.h>
#include <std_msgs/String.h>
#include <cctype>
#include <algorithm>

namespace BT
{
DotBt::DotBt(ros::NodeHandle* nh, TreeNode* root, const std::string& topic, const std::string& colorTopic, int tickperiod)
{
    this->root_ = root;
    this->topic_ = topic;
    this->colorUpdate_topic_ = colorTopic;
    this->n_ = nh;
    dotbt_publisher_ = n_->advertise<std_msgs::String>(topic_, 1, true);
    dotbt_color_publisher_ = n_->advertise<sts_bt_library::bt_draw_update>(colorUpdate_topic_,1,true);
    toggle_ = false;
    trigger_ = false;
    preempt = false;

    waitTime_ = double(tickperiod) / 10000;
    std::cout << "loop_rate " << waitTime_ << std::endl;

    //ROS_INFO_STREAM("Visualization: Start publishing the tree in topic: "
    //    << topic_ << " with rate: " << ros_rate << " Hz.");
}

DotBt::~DotBt()
{}

std::string DotBt::normaliseString(std::string name)
{
    std::string nn(name);

    for(unsigned int i = 0; i < nn.length(); i++)
    {
        if (nn[i] == '/')
        {
            nn.erase(i,1);
            nn.insert(i,"__");
        }
        else if(nn[i] == '!')
        {
            nn.erase(i,1);
            nn.insert(i,"not_");
        }
    }
    return nn;
}

std::string DotBt::defineNodeDot(TreeNode* node, bool root)
{
    std::string output = node->get_id();

    // Find the type of the node and its shape and symbol (label).
    switch (node->DrawType())
    {
    case SELECTORSTAR:
        output += "[label=\"?*\", penwidth=2, shape=box,";
        break;
    case BT::SEQUENCESTAR:
        output += "[label=\"->*\", penwidth=2,  shape=box,";
        break;
    case BT::SELECTOR:
        output += "[label=\"?\", penwidth=2,  shape=box,";
        break;
    case BT::SEQUENCE:
        output += "[label=\"->\", penwidth=2,  shape=box,";
        break;
    case BT::PARALLEL:
        output += "[label=\"->\n->\", penwidth=2,  shape=box,";
        break;
    case BT::DECORATOR:
        output += "[label=D, penwidth=2, shape=diamond,";
        break;
    case BT::ACTION:
        output += "[label=\"" + normaliseString(node->get_name()) + "\", penwidth=2, shape=box, rank=max";
        break;
    case BT::CONDITION:
        output += "[label=\"" + normaliseString(node->get_name()) + "\", penwidth=2, shape=ellipse, rank=max";
        break;
    default:
        output += "[label=\"" + normaliseString(node->get_name()) + "\", penwidth=2, shape=box,";
        break;
    }

    // Get the current status of the node for the coloring.
    switch (node->get_color_status())
    {
    case BT::RUNNING:
        output += " color=midnightblue";
        break;
    case BT::SUCCESS:
        output += " color=green";
        break;
    case BT::FAILURE:
        output += " color=red";
        break;
    case BT::IDLE:
        output += " color=black";
        break;
    case BT::HALTED:
        output += " color=orange";
        break;
    default:
        output += " color=black";
        break;
    }

    output += "]";

    return output;
}

void DotBt::produceDot(TreeNode* node, TreeNode* parent, bool insertInvisible, bool setEdgeWeight)
{
    bool root = false;

    //Check if insert invisible
    if (insertInvisible)
    {
        dot_file_ += node->get_id() +"_invis [style=invis] " + parent->get_id() + " -> " + node->get_id() + "_invis [style=invis weight=10000]\n";
        subgraph_ += node->get_id() +"_invis\n";
    }

    // If this node is the root of the tree initialize the directed graph
    if (parent == NULL)
    {
        //prefix is added in sts_bt_visualisation
        dot_file_ = ""; //"digraph debug_bt { \n";
        subgraph_ = "";
        root = true;
    }
    else
    {
        //found new subgraph
        if (parent == root_)
        {
            //if not  the first subgraph save it to dot language
            if (subgraph_ != "")
            {
                subgraph_ += "}";
                dot_file_ += "\n" + subgraph_ + "\n";
            }
            //Write new subgraph
            subgraph_ = "subgraph cluster_" + node->get_id() + "{\n" + node->get_id() + "\n";
        }
        else
        {
            //Add the current node also to the subgraph
            subgraph_ += node->get_id() + "\n";
        }
    }

    // Add the definition of this node
    dot_file_ += defineNodeDot(node, root) + " \n";

    // If the node has a parent, add it as a child of its parent.
    if (parent != NULL)
    {
        if (setEdgeWeight)
            dot_file_ += parent->get_id() + " -> " + node->get_id() + " [weight = 10000]\n";
        else
            dot_file_ += parent->get_id() + " -> " + node->get_id() + " \n";
    }

    // If this node has children run recursively for each child.
    BT::ControlNode* n = dynamic_cast<BT::ControlNode*> (node);
    if (n != NULL)
    {
        std::vector<TreeNode *> children = n->GetChildren();

        //create additional spacing at 0 child
        if (node != root_ && children.size() > 0)
        {
            dot_file_ += node->get_id() +"_0 [style=invis] " + node->get_id() + " -> " + node->get_id() + "_0 [style=invis]\n";
            subgraph_ += node->get_id() +"_0\n";
        }

        for (unsigned int i = 0; i < children.size(); i++)
        {
            if (children.size() == 2 && children.size() % 2 == 0 && i == children.size() / 2)
                produceDot(children.at(i), node, true, false);
            else if (children.size() % 2 == 1 && i == children.size() / 2)
                produceDot(children.at(i), node, false, true);
            else
                produceDot(children.at(i), node, false, false);
        }

        //create additional spacing at last child
        //dot_file_ += node->get_id() +"_m [style=invis] " + parent->get_id() + " -> " + node->get_id() + "_m [style=invis]\n";
        //subgraph_ += node->get_id() +"_m\n";
    }

    // In case every recursive calls returns to the root call, close the file.
    if (parent == NULL)
    {
        dot_file_ += subgraph_ + "} \n}";
    }
}

std::string DotBt::getDotFile()
{
    return dot_file_;
}

void DotBt::publish()
{
    std_msgs::String msg;

    //Publish tree structure once
    produceDot(root_, NULL, false, false);
    msg.data = dot_file_;
    dotbt_publisher_.publish(msg);

    // Start the loop for publishing the tree
    while (ros::ok() && !this->preempt)
    {
        //Publish color update every trigger
        if (trigger_)
        {
            toggleHeartBeat();

            std::lock_guard<std::mutex> guard(pushEventMsgMutex_);
            dotbt_color_publisher_.publish(updateMsg_);

            updateMsg_.updateList.clear();
            trigger_ = false;
        }
        ros::spinOnce();
        ros::Duration(waitTime_).sleep();
    }
}

void DotBt::triggerPublish()
{
    trigger_ = true;
}

void DotBt::triggerDestruction()
{
    this->preempt = true;
}

void DotBt::toggleHeartBeat()
{
    std::string colorString;
    sts_bt_library::bt_draw_event eventMsg;
    eventMsg.event = 1;
    eventMsg.target = root_->get_id();
    eventMsg.attribute = "color";

    if (toggle_)
    {
        eventMsg.value = "violet";
        toggle_ = false;
    }
    else
    {
        eventMsg.value = "black";
        toggle_ = true;
    }

    std::lock_guard<std::mutex> guard(pushEventMsgMutex_);
    updateMsg_.updateList.push_back(eventMsg);
}

void DotBt::statusChangedEvent(BT::TreeNode* node, BT::ReturnStatus status)
{
    std::string colorString;
    sts_bt_library::bt_draw_event eventMsg;
    eventMsg.event = 0;
    eventMsg.target = node->get_id();
    eventMsg.attribute = "color";

    switch (status)
    {
    case BT::RUNNING:
        colorString = "midnightblue";
        break;
    case BT::SUCCESS:
        colorString = "green";
        break;
    case BT::FAILURE:
        colorString = "red";
        break;
    case BT::IDLE:
        colorString = "black";
        break;
    case BT::HALTED:
        colorString = "orange";
        break;
    default:
        colorString = "black";
        break;
    }

    eventMsg.value = colorString;

    std::lock_guard<std::mutex> guard(pushEventMsgMutex_);
    updateMsg_.updateList.push_back(eventMsg);
}

}  // namespace BT
