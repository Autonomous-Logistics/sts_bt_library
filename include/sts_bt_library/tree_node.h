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

#pragma once

#ifndef TREE_NODE_H
#define TREE_NODE_H


/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST


   #define DEBUG  // uncomment this line if you want to print debug messages

#ifdef DEBUG
  // #define DEBUG_STDERR(x) (std::cerr << (x))
#define DEBUG_STDOUT(str) do { std::cout << str << std::endl; } while ( false )  // NOLINT(whitespace/braces)


#else
#define DEBUG_STDOUT(str)
#endif


#include <iostream>
#include <unistd.h>

#include <string>

#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <sts_bt_library/exceptions.h>
#include <sts_bt_library/bt_draw_update.h>
#include <sts_bt_library/logger.h>

namespace BT
{
// Enumerates the possible types of a node, for drawinf we have do discriminate whoich control node it is:

enum NodeType {ACTION_NODE, CONDITION_NODE, CONTROL_NODE};
enum DrawNodeType {PARALLEL, SELECTOR, SEQUENCE, SEQUENCESTAR, SELECTORSTAR, ACTION, CONDITION, DECORATOR, FALSE};
enum class HookActionType {SET, RESET, GET};
// Enumerates the states every node can be in after execution during a particular
// time step:
// - "Success" indicates that the node has completed running during this time step;
// - "Failure" indicates that the node has determined it will not be able to complete
//   its task;
// - "Running" indicates that the node has successfully moved forward during this
//   time step, but the task is not yet complete;
// - "Idle" indicates that the node hasn't run yet.
// - "Halted" indicates that the node has been halted by its father.
enum ReturnStatus {RUNNING, SUCCESS, FAILURE, IDLE, HALTED, EXIT};

// Enumerates the options for when a parallel node is considered to have failed:
// - "FAIL_ON_ONE" indicates that the node will return failure as soon as one of
//   its children fails;
// - "FAIL_ON_ALL" indicates that all of the node's children must fail before it
//   returns failure.
enum FailurePolicy {FAIL_ON_ONE, FAIL_ON_ALL};
enum ResetPolity   {ON_SUCCESS_OR_FAILURE, ON_SUCCESS, ON_FAILURE};

// Enumerates the options for when a parallel node is considered to have succeeded:
// - "SUCCEED_ON_ONE" indicates that the node will return success as soon as one
//   of its children succeeds;
// - "BT::SUCCEED_ON_ALL" indicates that all of the node's children must succeed before
//   it returns success.
enum SuccessPolicy {SUCCEED_ON_ONE, SUCCEED_ON_ALL};


/// fgn: class for hookReturn Values
class HookReturn
{
public:
    void setVal(int val){this->val_ = val; this->valid_ = true;}
    int getVal(){return val_;}
    bool isValid(){return this->valid_;}

private:
    int val_ = 0;
    bool valid_ = false;
};

// forward declarations
class Event;

// Abstract base class for Behavior Tree Nodes
class TreeNode
{
public:
    explicit TreeNode(std::string id, std::string name, BT::Logger* logger);
    virtual ~TreeNode();

    ///fgn:
    /// will call all tick functions by using FULL pre/tick/post event handling
    virtual BT::ReturnStatus TickCycle(std::vector<TreeNode*> &children, std::size_t index, std::string& id);
    ///fgn:
    /// tick events the leafs can use
    virtual void PreTick() {}
    virtual void PostTick() {}
    /// fgn: method for hook trigger mechanism
    virtual BT::HookReturn Hook(BT::TreeNode* sender, BT::HookActionType type){(void)sender; (void)type; return BT::HookReturn();}
    /// The method that is going to be executed when the node receive a tick
    virtual BT::ReturnStatus Tick(std::string& id) = 0;
    /// fgn: method to sync input buffers
    virtual void Sync() = 0;
    virtual void Halt() = 0;
    virtual void ResetColorState() = 0;
    virtual int DrawType() = 0;
    virtual int Depth() = 0;

    // The method used for getting all children
    virtual std::vector<TreeNode*> GetChildren();
    // The method used to fill the child vector
    virtual void AddChild(TreeNode* child);
    // The method used to fill the hook vector
    virtual void AddHook(TreeNode* child);

    // The method that retrive the state of the node
    // (conditional waiting and mutual access)
    // ReturnStatus GetNodeState();
    void SetNodeState(ReturnStatus new_state);
    void set_color_status(ReturnStatus new_color_status);
    // Methods used to access the node state without the
    // conditional waiting (only mutual access)
    ReturnStatus ReadState();
    ReturnStatus get_color_status();
    virtual std::size_t GetChildrenNumber();
    //  Getters and setters
    void set_x_pose(float x_pose);
    float get_x_pose();
    void set_x_shift(float x_shift);
    float get_x_shift();
    ReturnStatus get_status();
    void set_status(ReturnStatus new_status);
    std::string get_id();
    void set_id(std::string new_id);
    std::string get_name();
    void set_name(std::string new_name);
    NodeType get_type();
    bool has_parent();
    void set_has_parent(bool value);

    void pushEvent(int id, std::string msg);


protected:
    // Children vector
    std::vector<TreeNode*> children_nodes_;
    // Hook vector
    std::vector<TreeNode*> hooks_;
    std::string id_;
    std::string name_;
    bool has_parent_;
    BT::Logger* logger_;

    // The node state that must be treated in a thread-safe way
    bool is_state_updated_;
    ReturnStatus status_;
    ReturnStatus color_status_;

    std::mutex state_mutex_;
    std::mutex color_state_mutex_;
    std::condition_variable state_condition_variable_;
    // Node type
    NodeType type_;
    // position and offset for horizontal positioning when drawing
    float x_shift_, x_pose_;
};
}  // namespace BT

#endif  // TREE_NODE_H
