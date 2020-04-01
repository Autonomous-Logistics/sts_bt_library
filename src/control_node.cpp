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


#include <sts_bt_library/control_node.h>
#include <string>
#include <vector>

BT::ControlNode::ControlNode(std::string id, std::string name, BT::Logger* logger) : TreeNode::TreeNode(id, name, logger)
{
    type_ = BT::CONTROL_NODE;

    // TODO(...) In case it is desired to set to idle remove the ReturnStatus
    // type in order to set the member variable
    // ReturnStatus child_i_status_ = BT::IDLE;  // commented out as unused
}

BT::ControlNode::~ControlNode()
{
    //Delete all children recursively
    std::vector<BT::TreeNode*> childrenVector =  this->GetChildren();

    for (unsigned int i = 0; i < childrenVector.size(); i++)
    {
        delete childrenVector[i];
    }
}

void BT::ControlNode::Halt()
{
    HaltChildren(0);
    set_status(BT::HALTED);
}

void BT::ControlNode::ResetColorState()
{
    this->set_color_status(BT::IDLE);
    for (unsigned int i = 0; i < children_nodes_.size(); i++)
    {
        children_nodes_[i]->ResetColorState();
    }
}


int BT::ControlNode::Depth()
{
    int depMax = 0;
    int dep = 0;
    for (unsigned int i = 0; i < children_nodes_.size(); i++)
    {
        dep = (children_nodes_[i]->Depth());
        if (dep > depMax)
        {
            depMax = dep;
        }
    }
    return 1 + depMax;
}


void BT::ControlNode::HaltChildren(std::size_t i)
{
    for (unsigned int j=i; j < children_nodes_.size(); j++)
    {
        if (children_nodes_[j]->get_type() == BT::CONDITION_NODE)
        {
            children_nodes_[i]->ResetColorState();
        }
        else
        {
            if (children_nodes_[j]->get_status() == BT::RUNNING)
            {
                //std::cout << "SENDING HALT TO CHILD " << children_nodes_[j]-> get_name();
                children_nodes_[j]->Halt();
            }
            else
            {
                //DEBUG_STDOUT("NO NEED TO HALT " << children_nodes_[j]-> get_name()
                            // << " STATUS " << children_nodes_[j]->get_status());
            }
        }
    }
}

/// fgn: if you get a sync call, sync all children instead
void BT::ControlNode::Sync()
{
    for (unsigned int i=0; i < children_nodes_.size(); i++)
    {
        children_nodes_[i]->Sync();
    }
}
