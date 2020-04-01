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

#include <sts_bt_library/flip_flop_node.h>
#include <string>


BT::FlipFlopNode::FlipFlopNode(std::string id, std::string name, BT::Logger* logger) : BT::LeafNode(id, name, logger)
{
    //Init to false
    this->memory_ = false;
}

BT::FlipFlopNode::~FlipFlopNode()
{}

BT::ReturnStatus BT::FlipFlopNode::Tick(std::string& id){ return BT::SUCCESS; }
int BT::FlipFlopNode::DrawType(){ return BT::DrawNodeType::ACTION; }

BT::HookReturn BT::FlipFlopNode::Hook(BT::TreeNode* sender, BT::HookActionType type)
{
    //std::cout << "hook "<< this->get_name()<< ": " << sender->get_name() << " --> type: " << (int)type << std::endl;
    BT::HookReturn ret;
    switch(type)
    {
    case BT::HookActionType::SET:
        this->memory_ = true;
        break;
    case BT::HookActionType::RESET:
        this->memory_ = false;
        break;
    case BT::HookActionType::GET:
        /// we could implement this, but for now i will test if the return value thingy is enough!
        //sender->hookResult(BT::HookNodeType FLIPFLOP, memory_);
        ret.setVal(this->memory_);

        break;
    default:
        break;
    }

    return ret;
}
