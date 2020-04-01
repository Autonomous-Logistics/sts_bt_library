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

#include <sts_bt_library/parallel_node.h>
#include <string>

BT::ParallelNode::ParallelNode(std::string id, std::string name, BT::Logger* logger) : ControlNode::ControlNode(id, name, logger)
{

}

BT::ParallelNode::~ParallelNode()
{}

BT::ReturnStatus BT::ParallelNode::Tick(std::string& id)
{
    //std::cout << "parallel ticked " << this->get_status() << " ###\n";
    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    std::size_t N_of_children_ = this->children_nodes_.size();

    /// define default status which we use for ourselfs and which is returned
    BT::ReturnStatus parallelStatus = BT::RUNNING;

    /// [INIT] define new vectory for holding our children states if we were ticked for the first time
    if(this->get_status() == BT::IDLE || this->get_status() == BT::HALTED)
        this->children_states_ = std::vector<BT::ReturnStatus>(N_of_children_);

    // Routing the tree according to the sequence node's logic:
    for (unsigned int i = 0; i < N_of_children_; i++)
    {
        /// tick that child and update our memory with its result
        this->children_states_[i] = this->TickCycle(this->children_nodes_, i, id);
    }
    /// check all our current children results if we can leave properly
    if(this->isFinished(parallelStatus))
    {
        /// isFinished will set the resulting Status if it returns true
        /// so now we have our return status defined by our children behavior
        /// and we also know that we are done here .. so prepare leave
        this->Reset();
    }

    this->set_status(parallelStatus);
    //std::cout << "parallel returning " << parallelStatus  << " >> " << this->get_status() << " ###\n";
    return parallelStatus;
}

bool BT::ParallelNode::isFinished(BT::ReturnStatus &nodeStatus)
{
    //std::cout << "isFinished()\n";
    bool finished = false;
    unsigned int n = this->children_states_.size();
    /// create default isFailure and isSuccess tokens, these have to be set XOR mutually exclusive
    bool isSuccess = true;
    bool isFailure = false;
    bool isRunning = false;
    for (unsigned int i = 0; i < n; i++)
    {
        BT::ReturnStatus state = children_states_[i];
        //std::cout << "s[" << i << "] = " << state << ", ";
        if(state == BT::SUCCESS)
        {
            isSuccess = isSuccess & true;
        }
        else if(state == BT::FAILURE)
        {
            isFailure = isFailure | true;
        }
        else if(state == BT::RUNNING)
        {
            isRunning = true;
        }
    }

    /// check if all childs are SUCCESS or FAILURE and react that way
    if(isRunning)
    {
        /// if we have no definitive answer yet, we are probably running?!
        /// do we want to preempt our running early when a single child is FAILURE?
        //if(isFailure)
        //  do stuff for example:
        //this->HaltChildren(0);  // halts all running children. The execution is hopeless.
    }
    else
    {
        /// we are probably done here ...
        finished = true;
        /// how are we done exactly?
        if(isFailure)
        {
            //std::cout << "... done fail\n";
            /// if one of the nodes has failed ... we are done with status FAILURE ...
            /// we also have to reset whatever is still RUNNING
            nodeStatus = BT::FAILURE;
        }
        else if(isSuccess)
        {
            //std::cout << "... done success\n";
            /// if all nodes are successful ... we are done with status SUCCESS ...
            nodeStatus = BT::SUCCESS;
        }
    }
    return finished;
}

void BT::ParallelNode::Reset()
{
    /// clear our children states internally
    this->children_states_.clear();
    /// reset children states externally (for real)
    for (unsigned int i = 0; i < this->GetChildrenNumber(); i++)
    {
        BT::ReturnStatus childStatus = this->children_nodes_[i]->get_status();
        //if not IDLE or HALTED reset status
        if (childStatus != BT::IDLE && childStatus != BT::HALTED)
            this->children_nodes_[i]->set_status(BT::IDLE);
    }
}

void BT::ParallelNode::Halt()
{
    BT::ControlNode::Halt();
    this->Reset();
}

int BT::ParallelNode::DrawType()
{
    return BT::PARALLEL;
}




