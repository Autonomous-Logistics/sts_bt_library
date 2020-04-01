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

#pragma once

//ROS
#include <actionlib/client/simple_action_client.h>
//behavior tree
#include <sts_bt_library/leaf_node.h>
#include <sts_bt_library/control_node.h>
//sts_virtual_interfaces
#include <sts_virtual_interface_msgs.h>
#include <sts_virtual_interface_msgs/BTAction.h>
#include <sts_virtual_interface_msgs/BTService.h>


#include <std_msgs/Bool.h>
#include <thread>
#include <chrono>

//Idea HM timeouts realtime
//enum CriticalityEnum{Critical, Moderate, Negligible}; // Idea for health monitoring / different tree behavior like reticks and time to notice HM

class sts_bt_if_node : public BT::LeafNode //BT::ControlNode
{
public:
    sts_bt_if_node(std::string id, std::string name, BT::Logger* logger);
    virtual ~sts_bt_if_node();

    /// must have virtual override functions
    virtual BT::ReturnStatus Tick(std::string& id) = 0;
    virtual int DrawType();

public:
    virtual void initialize(std::string id, std::string rosName, bool condition,  ros::NodeHandle* n, bool errorDefault, bool inverted);

    void setName(std::string name) {this->name_ = name;}
    std::string getName() {return this->name_;}
    void setRosName(std::string name) {this->nameRos_ = name;}
    std::string getRosName() {return this->nameRos_;}

    void setCondition(bool condition) {this->condition_ = condition;}
    bool getCondition() {return this->condition_;}

    void setNodeHandle(ros::NodeHandle* n) {this->n_ = n;}
    ros::NodeHandle* getNodeHandle() {return this->n_;}

    void setErrorDefault(bool errorDefault) {this->errorDefault_ = errorDefault;}
    bool getErrorDefault() {return this->errorDefault_;}

    void setInvertedLogic(bool inverted) {this->invertedLogic_ = inverted;}
    bool getInvertedLogic() { return this->invertedLogic_;}

protected:
    std::string normalizeToRosName(const std::string& name);


private:
    std:: string name_;
    std:: string nameRos_;
    bool condition_;
    ros::NodeHandle* n_;
    bool errorDefault_;
    bool invertedLogic_;
};


//####################################
//############# TOPIC ################
//####################################
class sts_bt_if_node_topic : public sts_bt_if_node
{
public:
    sts_bt_if_node_topic(std::string id, std::string name, BT::Logger* logger);
    ~sts_bt_if_node_topic();

    /// must have virtual override functions
    BT::ReturnStatus Tick(std::string& id);
    virtual void Sync();

public:
    void initialize(std::string id, std::string rosName, bool condition,  ros::NodeHandle* n, bool errorDefault, bool inverted);

private:
    void topicCallback(const std_msgs::Bool message);
    bool readInput();
    void enableInput(bool enable);
    void writeInput(bool input);

private:
    ros::Subscriber subscriber_;
    //const unsigned int BUFLEN_ = 1;
    //const unsigned int INMASK_ = 1 << BUFLEN_;
    //unsigned int messageBuffer_ = 0;
    bool receiveBuffer_ = false;
    bool dataBuffer_ = false;
    bool outputBuffer_ = false;
};


//##################################
//############# SERVICE ############
//##################################

class sts_bt_if_node_service : public sts_bt_if_node
{
public:
    const static unsigned int MAX_SERVICE_RETRYS = 2;

    sts_bt_if_node_service(std::string id, std::string name, BT::Logger* logger);
    ~sts_bt_if_node_service();

    /// must have virtual override functions
    BT::ReturnStatus Tick(std::string& id);
    virtual void Sync();

public:
    void initialize(std::string id, std::string rosName, bool condition,  ros::NodeHandle* n, bool errorDefault, bool inverted);

private:
    ros::ServiceClient client_;
    sts_virtual_interface_msgs::BTService srv_;
};

//#################################
//############# ACTION ############
//#################################

class sts_bt_if_node_action : public sts_bt_if_node
{
public:
    sts_bt_if_node_action(std::string id, std::string rosName, BT::Logger *logger);
    ~sts_bt_if_node_action();

    /// must have virtual override functions
    virtual void PreTick();
    virtual BT::ReturnStatus Tick(std::string& id);
    virtual void PostTick();
    virtual BT::HookReturn Hook(BT::TreeNode* sender, BT::HookActionType type);
    virtual void Halt();

public:
    void initialize(std::string id, std::string rosName, bool condition,  ros::NodeHandle* n, bool errorDefault, bool inverted);

private:
    void actionResultCallback(const actionlib::SimpleClientGoalState& state, const sts_virtual_interface_msgs::BTResultConstPtr& result);
    BT::ReturnStatus generateBtResult(int32_t btResultStatus);
    void connectAS();

private:
    actionlib::SimpleActionClient<sts_virtual_interface_msgs::BTAction> ac_;
    std::thread connectASThread;
    bool actionComInit;
    bool preempt_ = false;
    bool running_ = false;
    sts_virtual_interface_msgs::BTGoal goal_;
    sts_virtual_interface_msgs::BTFeedback feedback_;
    sts_virtual_interface_msgs::BTResult result_;


};

//##################################
//############# FACTORY ############
//##################################

//typedef std::shared_ptr<sts_bt_if_node> StsBtIfNodePtr;
typedef sts_bt_if_node* StsBtIfNodePtr;

class sts_bt_if_factory
{
public:
    enum comTypeEnum{TOPIC, SERVICE, ACTION};

    static StsBtIfNodePtr createNode(std::string id, std::string rosName, bool condition, comTypeEnum comType, ros::NodeHandle* n, bool errorDefault, BT::Logger* logger);
};
