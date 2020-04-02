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

#include <sts_bt_library/fallback_node.h>
#include <string>

BT::FallbackNode::FallbackNode(std::string id, std::string name, BT::Logger* logger) : ControlNode::ControlNode(id, name, logger) {}

BT::FallbackNode::~FallbackNode()
{}

BT::ReturnStatus BT::FallbackNode::Tick(std::string& id)
{
    {
        // gets the number of children. The number could change if, at runtime, one edits the tree.
        unsigned int N_of_children_ = children_nodes_.size();
        // Routing the ticks according to the fallback node's logic:
        for (unsigned int i = 0; i < N_of_children_; i++)
        {
            /*      Ticking an action is different from ticking a condition. An action executed some portion of code in another thread.
                    We want this thread detached so we can cancel its execution (when the action no longer receive ticks).
                    Hence we cannot just call the method Tick() from the action as doing so will block the execution of the tree.
                    For this reason if a child of this node is an action, then we send the tick using the tick engine. Otherwise we call the method Tick() and wait for the response.
            */

            // 1) Send the tick and wait for the response;
            child_i_status_ = this->TickCycle(children_nodes_, i, id);

            // Ponderate on which status to send to the parent
            if (child_i_status_ != BT::FAILURE)
            {
                if (child_i_status_ == BT::SUCCESS)
                {
                    children_nodes_[i]->set_status(BT::IDLE);  // the child goes in idle if it has returned success.
                }
                // If the  child status is not failure, halt the next children and return the status to your parent.
                HaltChildren(i+1);
                set_status(child_i_status_);
                return child_i_status_;
            }
            else
            {
                // the child returned failure.
                children_nodes_[i]->set_status(BT::IDLE);
                if (i == N_of_children_ - 1)
                {
                    // If the  child status is failure, and it is the last child to be ticked,
                    // then the sequence has failed.
                    set_status(BT::FAILURE);

                    return BT::FAILURE;
                }
            }
        }
    }

    return BT::EXIT;
}

int BT::FallbackNode::DrawType()
{
    // Lock acquistion

    return BT::SELECTOR;
}
