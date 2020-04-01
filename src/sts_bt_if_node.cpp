/* Copyright (C) 2019-2020 Sebastian Mueller, Nick Fiege, Robert Rudolph - All Rights Reserved
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
#include "sts_bt_library/sts_bt_if_node.h"

sts_bt_if_node::sts_bt_if_node(std::string id, std::string name, BT::Logger* logger) : LeafNode(id, name, logger) {}

sts_bt_if_node::~sts_bt_if_node()
{}

void sts_bt_if_node::initialize(std::string id, std::string rosName, bool condition, ros::NodeHandle* n, bool errorDefault, bool inverted)
{
    this->setName(id);
    this->setRosName(this->normalizeToRosName(rosName));

    this->setCondition(condition);
    this->setNodeHandle(n);
    this->setErrorDefault(errorDefault);
    this->setInvertedLogic(inverted);

    if (this->getCondition())
        this->type_ = BT::CONDITION_NODE;
    else
        this->type_ = BT::ACTION_NODE;
}

std::string sts_bt_if_node::normalizeToRosName(const std::string &name)
{
    std::string nn(name);
    /// remove unnecessary things
    if(nn[0] == '!') nn = nn.substr(1);
    ///add namespace if no forward "/"
    std::string ns = "";
    if(nn[0] != '/')
    {
        ns = sts_core::sts_interfaces::BEHAVIOR_TREE_NAMESPACE;
        //this->getNodeHandle()->getParam(sts_core::sts_interfaces::BEHAVIOR_TREE_NAMESPACE_KEY, ns);
        nn = ns + "/" + nn;
    }

    //clean newlines
    nn.erase(std::remove(nn.begin(), nn.end(), '\n'), nn.end());

    return nn;
}

int sts_bt_if_node::DrawType()
{
    if (this->getCondition())
        return BT::CONDITION;
    else
        return BT::ACTION;
}

//####################################
//############# TOPIC ################
//####################################

sts_bt_if_node_topic::sts_bt_if_node_topic(std::string id, std::string name, BT::Logger* logger) : sts_bt_if_node(id, name, logger) {}//: BASE {}

sts_bt_if_node_topic::~sts_bt_if_node_topic(){}

void sts_bt_if_node_topic::initialize(std::string id, std::string rosName, bool condition, ros::NodeHandle *n, bool errorDefault, bool inverted)
{
    //Base class initialization
    sts_bt_if_node::initialize(id, rosName, condition, n, errorDefault, inverted);

    //Local initialization
    //std::cout << "[BT*] REGISTERING COM [TOPIC]   " << "C>"<<condition << ": " << this->getRosName() << "\n";
    subscriber_ = this->getNodeHandle()->subscribe(this->getRosName(), 1, &sts_bt_if_node_topic::topicCallback, this);
    this->outputBuffer_ = this->getErrorDefault();
    this->dataBuffer_ = this->getErrorDefault();
    this->receiveBuffer_ = this->getErrorDefault();
}


BT::ReturnStatus sts_bt_if_node_topic::Tick(std::string& id)
{
    /// Set the name of this node to return it as last called by the tree
    id = this->get_name();

    /// Ask [enable signal] hooks for permission to apply input in inputBuffer (enable)
    bool enable = true;
    for(auto node : this->hooks_)
    {
        BT::HookReturn ret = node->Hook(this, BT::HookActionType::GET);
        if(ret.isValid())
        {
            enable = enable & ret.getVal();
            if(!enable)
                break;
        }
    }

    /// if we are allowed to output our stuff properly, shift dataBuffer_ into outputBuffer_
    this->enableInput(enable);

    ///[fgn] apply inverted flag
    /* truth table for logic --> XOR
     *  |-----|-----|-----------|
        | Buf | Inv |  SUCCESS  |
        |-----|-----|-----------|
        |  0  |  0  |   False   |
        |-----|-----|-----------|
        |  0  |  1  |   True    |
        |-----|-----|-----------|
        |  1  |  0  |   True    |
        |-----|-----|-----------|
        |  1  |  1  |   False   |
        |-----|-----|-----------|
    */
    if (this->readInput() ^ this->getInvertedLogic())
    {
        //std::cout << "ticked ... SUCCESS \n";
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else
    {
        //std::cout << "ticked ... FAILURE \n";
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
}

void sts_bt_if_node_topic::topicCallback(const std_msgs::Bool message)
{
    /// set message buffer input to newest value
    this->writeInput(message.data);
}


bool sts_bt_if_node_topic::readInput()
{
    /// read our actual accepted value
    return this->outputBuffer_;
}
void sts_bt_if_node_topic::enableInput(bool enable)
{
    if(enable)
    {
        //std::cout << "enabling input from " << this->outputBuffer_ << " to " << this->dataBuffer_ << "\n";
        /// apply our current synced input to our accepted value
        this->outputBuffer_ = this->dataBuffer_;
    }
}

void sts_bt_if_node_topic::writeInput(bool input)
{
    /// register = (register & ~(maskBitNumber)) | input << bufferSize
    // this->messageBuffer_ = (this->messageBuffer_ & ~(this->INMASK_)) | input << this->BUFLEN_;
    this->receiveBuffer_ = input;
}
void sts_bt_if_node_topic::Sync()
{
    /// we got the command to sync our input register to the latest received value
    this->dataBuffer_ = this->receiveBuffer_;
}



//##################################
//############# SERVICE ############
//##################################
sts_bt_if_node_service::sts_bt_if_node_service(std::string id, std::string name, BT::Logger* logger) : sts_bt_if_node(id, name, logger) {}//: BASE {}

sts_bt_if_node_service::~sts_bt_if_node_service(){}

void sts_bt_if_node_service::initialize(std::string id, std::string rosName, bool condition, ros::NodeHandle *n, bool errorDefault, bool inverted)
{
    //Base class initialization
    sts_bt_if_node::initialize(id, rosName, condition, n, errorDefault, inverted);

    //Local initialization
    //std::cout << "[BT*] REGISTERING COM [SERVICE] " << "C>"<<condition << ": " << this->getRosName() << "\n";
    client_ = this->getNodeHandle()->serviceClient<sts_virtual_interface_msgs::BTService>(this->getRosName(), true);
}

BT::ReturnStatus sts_bt_if_node_service::Tick(std::string& id)
{
    //Set the name of this node to return it as last called by the tree
    id = this->get_name();

    srv_.request.goal = true;

    BT::ReturnStatus state = BT::FAILURE;
    bool srvSuccesful = false;
    /// fgn: we could ask if the service is available, but that would be slow [i guess]
    //bool serviceAvailable = ros::service::exists(this->getRosName(), false);

    ///[fgn] apply inverted flag
    /* truth table for logic --> XOR
     *  |-----|-----|-----------|
        | Buf | Inv |  SUCCESS  |
        |-----|-----|-----------|
        |  0  |  0  |   False   |
        |-----|-----|-----------|
        |  0  |  1  |   True    |
        |-----|-----|-----------|
        |  1  |  0  |   True    |
        |-----|-----|-----------|
        |  1  |  1  |   False   |
        |-----|-----|-----------|
    */
    //auto start = std::chrono::high_resolution_clock::now();

    //if(serviceAvailable)
    {
        int timeout = -1;
        while(timeout < (int)sts_bt_if_node_service::MAX_SERVICE_RETRYS)
        {
            /// do service call and jump out if we were sucessful
            if (client_.call(srv_))
            {
                if ((srv_.response.result == true) ^ this->getInvertedLogic())
                    state = BT::SUCCESS;
                else
                    state = BT::FAILURE;
                /// we have done something usefull!
                srvSuccesful = true;
                break;
            }
            /// if we reach here, something went wrong ...
            /// check if service connection was dropped accidentally, reconnect if dropped
            if(!client_.isValid())
                client_ = this->getNodeHandle()->serviceClient<sts_virtual_interface_msgs::BTService>(this->getRosName(), true);
            /// inc timeout for max retrys
            timeout++;
        }
    }

    if(!srvSuccesful)
        std::cout << "Ticked service [" << this->getRosName() << "] not initialized - returning" << std::endl;

    /*
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";
    */
    this->set_status(state);
    this->pushEvent(state, "service done.");
    return state;
}
void sts_bt_if_node_service::Sync()
{
}


//#################################
//############# ACTION ############
//#################################

sts_bt_if_node_action::sts_bt_if_node_action(std::string id, std::string name, BT::Logger* logger) : sts_bt_if_node(id, name, logger), ac_(this->normalizeToRosName(name), true) {} // : BASE, ac_(name, true) {}

sts_bt_if_node_action::~sts_bt_if_node_action()
{
    this->preempt_ = true;
    if(this->connectASThread.joinable())
        this->connectASThread.join();
}

void sts_bt_if_node_action::initialize(std::string id, std::string rosName, bool condition, ros::NodeHandle *n, bool errorDefault, bool inverted)
{
    //Base class initialization
    sts_bt_if_node::initialize(id, rosName, condition, n, errorDefault, inverted);

    this->preempt_ = false;
    this->actionComInit = false;
    this->connectASThread = std::thread(&sts_bt_if_node_action::connectAS, this);

    //Local initialization
    //std::cout << "[BT*] REGISTERING COM [ACTION]  " << "C>"<<condition << ": " << this->getRosName() << "\n";
    //ac_.waitForServer(); //infinite wait time TODO: design boot sequence, health monitor information if server node not found etc.

    //Setting goal_ to running = 0, since only the trigger is needed
    goal_.parameter = 0;
}


void sts_bt_if_node_action::PreTick()
{
}

BT::ReturnStatus sts_bt_if_node_action::Tick(std::string& id)
{
    /// check if we are connected
    if (!this->actionComInit)
    {
        std::cout << "Ticked action [" << this->getRosName() << "] not initialized - returning" << std::endl;
        return this->generateBtResult(2); //1 = OK, 2 = FAIL
    }

    /// we only want to do something if we are idle
    if(this->get_status() == BT::IDLE)
    {
        //std::cout << "running " << this->getRosName() << "\n";
        this->running_ = true;
        this->pushEvent(0, "started");

        //Set the name of this node to return it as last called by the tree
        id = this->get_name();

        ac_.sendGoal(goal_, boost::bind(&sts_bt_if_node_action::actionResultCallback, this, _1, _2));
        set_status(BT::RUNNING);
    }
    return this->get_status();
}

void sts_bt_if_node_action::PostTick()
{
    /// if we are not runnign anymore because our action finished  (either halted or result via node side)
    if(this->get_status() != BT::RUNNING && this->get_status() != BT::HALTED)
    {
        //std::cout << "not running anymore ... " << this->getRosName() << " > " << (int)this->get_status() << "\n";
        /// supply enable signal
        this->running_ = false;
    }
}

BT::HookReturn sts_bt_if_node_action::Hook(BT::TreeNode* sender, BT::HookActionType type)
{
    bool finished = !this->running_;
    BT::HookReturn ret;
    switch(type)
    {
    case BT::HookActionType::GET:
        ret.setVal(finished);
        break;
    default:
        break;
    }

    return ret;
}


void sts_bt_if_node_action::actionResultCallback(const actionlib::SimpleClientGoalState& state, const sts_virtual_interface_msgs::BTResultConstPtr& result)
{
    /// this will be called if we returned normally (btResult SUCCESS/FAILURE)
    /// this will also be called if node preempted (btResult HALT)
    BT::ReturnStatus btResult = this->generateBtResult(result->status);
    /// if we received halted from a node, we want to set ourselfs to idle, because we were actually waiting for an action to cancel
    if(btResult == BT::HALTED)
    {
        //std::cout << "successfully halted " << this->getRosName() << ". \n";
        set_status(BT::IDLE);
    }
    /// else we an action returned normally, so we want to accept that result
    else
    {
        set_status(btResult);
    }
    this->pushEvent(btResult, "finished");
}

BT::ReturnStatus sts_bt_if_node_action::generateBtResult(int32_t btResultStatus)
{
    //if SUCCESS
    if (btResultStatus == 1)
    {
        if (this->getInvertedLogic()){return BT::FAILURE;}
        else {return BT::SUCCESS;}
    }
    //else if FAILURE))
    else if (btResultStatus == 2)
    {
        if (this->getInvertedLogic()){return BT::SUCCESS;}
        else {return BT::FAILURE;}
    }
    else if (btResultStatus == 4)
    {
        return BT::HALTED;
    }
    else {return BT::FAILURE;}

}


void sts_bt_if_node_action::Halt()
{
    std::cout << "Halting " << this->getRosName() << " ... \n";
    ac_.cancelGoal();
    set_status(BT::HALTED);
}

void sts_bt_if_node_action::connectAS()
{
    while(!this->preempt_ && !this->actionComInit)
        this->actionComInit = this->ac_.waitForServer(ros::Duration(0.01));
}

//##################################
//############# FACTORY ############
//##################################

StsBtIfNodePtr sts_bt_if_factory::createNode(std::string id, std::string name, bool condition, comTypeEnum comType, ros::NodeHandle* n, bool errorDefault, BT::Logger* logger)
{
    ///[fgn] check if this thing is inverted
    bool invertedLogic = false;
    if(name[0] == '!')
        invertedLogic = true;

    ///normal logic from now on
    if (comType == sts_bt_if_factory::TOPIC)
    {
        //std::shared_ptr<sts_bt_if_node_topic> sharedPtr(new sts_bt_if_node_topic(name));
        StsBtIfNodePtr sharedPtr(new sts_bt_if_node_topic(id, name, logger));
        sharedPtr->initialize(id, name,condition,n,errorDefault, invertedLogic);

        return sharedPtr;
    }
    else if (comType == sts_bt_if_factory::SERVICE)
    {
        //std::shared_ptr<sts_bt_if_node_service> sharedPtr(new sts_bt_if_node_service(name));
        StsBtIfNodePtr sharedPtr(new sts_bt_if_node_service(id, name, logger));
        sharedPtr->initialize(id, name,condition,n,errorDefault, invertedLogic);

        return sharedPtr;
    }
    else
    {
        //std::shared_ptr<sts_bt_if_node_action> sharedPtr(new sts_bt_if_node_action(name));
        StsBtIfNodePtr sharedPtr(new sts_bt_if_node_action(id, name, logger));
        sharedPtr->initialize(id, name,condition,n,errorDefault, invertedLogic);

        return sharedPtr;
    }
}
