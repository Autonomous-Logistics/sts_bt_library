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

#include <sts_bt_library/sequence_node_with_memory.h>
#include <string>


BT::SequenceNodeWithMemory::SequenceNodeWithMemory(std::string id, std::string name, BT::Logger* logger) : ControlNode::ControlNode(id, name, logger)
{
    reset_policy_ = BT::ON_SUCCESS_OR_FAILURE;
    current_child_idx_ = 0;  // initialize the current running child
}


BT::SequenceNodeWithMemory::SequenceNodeWithMemory(std::string id, std::string name, int reset_policy, BT::Logger* logger) : ControlNode::ControlNode(id, name, logger)
{
    reset_policy_ = reset_policy;
    current_child_idx_ = 0;  // initialize the current running child
}


BT::SequenceNodeWithMemory::~SequenceNodeWithMemory() {}


BT::ReturnStatus BT::SequenceNodeWithMemory::Tick(std::string& id)
{
    //DEBUG_STDOUT(get_name() << " ticked, memory counter: "<< current_child_idx_);

    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    unsigned int N_of_children_ = static_cast<unsigned int>(children_nodes_.size());

    // Routing the ticks according to the sequence node's (with memory) logic:
    while (current_child_idx_ < N_of_children_)
    {
        /*      Ticking an action is different from ticking a condition. An action executed some portion of code in another thread.
                We want this thread detached so we can cancel its execution (when the action no longer receive ticks).
                Hence we cannot just call the method Tick() from the action as doing so will block the execution of the tree.
                For this reason if a child of this node is an action, then we send the tick using the tick engine. Otherwise we call the method Tick() and wait for the response.
        */

        child_i_status_ = this->TickCycle(children_nodes_, current_child_idx_, id);

        /// define leaf node behavior
        if (child_i_status_ == BT::SUCCESS || child_i_status_ == BT::FAILURE )
        {
             // the child goes in idle if it has returned success or failure.
            children_nodes_[current_child_idx_]->set_status(BT::IDLE);
        }

        ///define control behavior
        if(child_i_status_ == BT::SUCCESS)
        {
            /// do we have the last child?
            if (current_child_idx_ == N_of_children_ - 1)
            {
                /// we are done, last child reached with success
                current_child_idx_ = 0;
            }
            else
            {
                /// we go to the next child
                current_child_idx_++;
                /// continue our loop, so that we dont return anything for now
                continue;
            }
        }
        else if(child_i_status_ == BT::FAILURE)
        {
            /// a child failed, reset ourselfs if policy allows it
            if(reset_policy_ == BT::ON_FAILURE || reset_policy_ == BT::ON_SUCCESS_OR_FAILURE)
            {
                current_child_idx_ = 0;
            }
        }

        /// set us to whatever the node is at all times
        this->set_status(child_i_status_);
        return child_i_status_;
    }

    return BT::EXIT;
}


int BT::SequenceNodeWithMemory::DrawType()
{
    return BT::SEQUENCESTAR;
}


void BT::SequenceNodeWithMemory::Halt()
{
    current_child_idx_ = 0;
    BT::ControlNode::Halt();
}
