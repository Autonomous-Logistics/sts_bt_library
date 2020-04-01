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

#include <sts_bt_library/tree_node.h>
#include <string>
#include <sts_bt_library/behavior_tree.h>
#include <sts_bt_library/Event.h>


BT::TreeNode::TreeNode(std::string id, std::string name, BT::Logger* logger)
{
    // Initialization
    id_ = id;
    name_ = name;
    has_parent_ = false;
    logger_ = logger;
    is_state_updated_ = false;
    set_status(BT::IDLE);
}

BT::TreeNode::~TreeNode()
{}

void BT::TreeNode::set_status(ReturnStatus new_status)
{
    if (new_status != BT::IDLE)
    {
        set_color_status(new_status);
    }

    // Lock acquistion
    std::unique_lock<std::mutex> UniqueLock(state_mutex_);

    // state_ update
    status_ = new_status;

    //Log state change
    if (this->logger_ != nullptr)
    {
        std::string enumStringConversion;
        std::string nodeName;
        nodeName = this->get_name();
        nodeName.erase(std::remove(nodeName.begin(), nodeName.end(), '\n'), nodeName.end());

        switch (status_)
        {
        case 0:
            enumStringConversion = "Running";
            break;
        case 1:
            enumStringConversion = "Success";
            break;
        case 2:
            enumStringConversion = "Failure";
            break;
        case 3:
            enumStringConversion = "Idle";
            break;
        case 4:
            enumStringConversion = "Halted";
            break;
        case 5:
            enumStringConversion = "Exit";
            break;
        default:
            enumStringConversion = "Unknown enum conversion";
            break;
        }
        std::string logStr = "(" + this->id_ + "," + nodeName + "," + enumStringConversion + ")";
        this->logger_->logRos(logStr);
    }
}

BT::ReturnStatus BT::TreeNode::get_status()
{
    // Lock acquistion
    //DEBUG_STDOUT(get_name() << " is setting its status to " << status_);

    std::lock_guard<std::mutex> LockGuard(state_mutex_);

    return status_;
}

BT::ReturnStatus BT::TreeNode::get_color_status()
{
    // Lock acquistion
    std::lock_guard<std::mutex> LockGuard(color_state_mutex_);

    return color_status_;
}

void BT::TreeNode::set_color_status(ReturnStatus new_color_status)
{
    if (new_color_status != color_status_)
        sts_behavior_tree::BehaviorTree::statusChangedEvent(this, new_color_status);

    // Lock acquistion
    std::lock_guard<std::mutex> LockGuard(color_state_mutex_);
    // state_ update
    color_status_ = new_color_status;
}

float BT::TreeNode::get_x_pose()
{
    return x_pose_;
}

void BT::TreeNode::set_x_pose(float x_pose)
{
    x_pose_ = x_pose;
}


float BT::TreeNode::get_x_shift()
{
    return x_shift_;
}

void BT::TreeNode::set_x_shift(float x_shift)
{
    x_shift_ = x_shift;
}

void BT::TreeNode::set_id(std::string new_id)
{
    id_ = new_id;
}
void BT::TreeNode::set_name(std::string new_name)
{
    name_ = new_name;
}

std::string BT::TreeNode::get_id()
{
    return id_;
}
std::string BT::TreeNode::get_name()
{
    return name_;
}


BT::NodeType BT::TreeNode::get_type()
{
    return type_;
}

bool BT::TreeNode::has_parent()
{
    return has_parent_;
}

void BT::TreeNode::set_has_parent(bool value)
{
    has_parent_ = value;
}

std::vector<BT::TreeNode*> BT::TreeNode::GetChildren()
{
    return this->children_nodes_;
}

std::size_t BT::TreeNode::GetChildrenNumber()
{
    return this->GetChildren().size();
}


void BT::TreeNode::AddChild(TreeNode* child)
{
    // Checking if the child has a parent already

    if (child->has_parent())
        throw BehaviorTreeException("'" + child->get_name() + " has a parent already. Please create different objects for multiple nodes. It makes the tinking/halting precedure easier.");

    child->set_has_parent(true);
    children_nodes_.push_back(child);
    //children_states_.push_back(BT::IDLE);
}

void BT::TreeNode::AddHook(TreeNode* child)
{
    this->hooks_.push_back(child);
}


BT::ReturnStatus BT::TreeNode::TickCycle(std::vector<TreeNode*> &children, std::size_t index, std::string& id)
{
    ///     preTick is called, before a tick of a leaf is executed
    ///     postTick is called, right before tick of the next neighbor is called
    /// ############################################################
    /// define some default things
    BT::ReturnStatus child_i_status_ = BT::IDLE;
    TreeNode* before = nullptr;
    TreeNode* current = nullptr;
    TreeNode* next = nullptr;
    std::size_t i = index;
    if(i > 0)    before = children[i-1];
    if(i < children.size()) current = children[i];
    if(i < children.size() - 1) next = children[i+1];
    /// ############################################################
    /// find out if we want to tick 'current'
    bool doTick = false;
    if(current != nullptr)
    {
        doTick = true;
        if(current->get_type() == BT::ACTION_NODE)
        {
            /// remember status of action, so that we return SUCCESS or FAILURE if necessary
            child_i_status_ = current->get_status();
            if(child_i_status_ != BT::IDLE && child_i_status_ != BT::HALTED)
                doTick = false;
        }
    }
    /// ############################################################
    /// use 'doTick' to trigger tickEngine
    /// call post tick event of i-1 if possible
    if(before != nullptr) {before->PostTick();}
    /// call pre tick event of i
    if(current != nullptr) {current->PreTick();}
    /// call regular tick event of i if node is idle or halted
    if(doTick) {
        id = current->get_name();
        child_i_status_ = current->Tick(id);
    }
    ///if we have reached the end of the children list, we want to call PostTick on the last remaining child aswell
    if(next == nullptr) {current->PostTick();}
    return child_i_status_;
}

void BT::TreeNode::pushEvent(int id, std::string msg)
{
    sts_behavior_tree::BehaviorTree::handleEvent(BT::Event(id, this, msg));
}
