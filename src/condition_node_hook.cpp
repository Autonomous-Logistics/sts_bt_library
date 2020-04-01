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

#include <sts_bt_library/condition_node_hook.h>
#include <string>

BT::ConditionNodeHook::ConditionNodeHook(std::string id, std::string name, BT::Logger* logger) : BT::LeafNode(id, name, logger)
{
    type_ = BT::CONDITION_NODE;
    /// check if we use inverted logic
    if(name[0] == '!')
        this->useInvertedLogic_ = true;

    ///default return value to tree
    if (this->useInvertedLogic_)
        this->defaultStatus_ = BT::ReturnStatus::SUCCESS;
    else
        this->defaultStatus_ = BT::ReturnStatus::FAILURE;
}

BT::ConditionNodeHook::~ConditionNodeHook()
{}

int BT::ConditionNodeHook::DrawType()
{ return BT::DrawNodeType::CONDITION; }


BT::ReturnStatus BT::ConditionNodeHook::Tick(std::string& id)
{
    BT::ReturnStatus status = this->defaultStatus_;
    ///go through all hooks and set them if necessary
    for(auto node : this->hooks_)
    {
        BT::HookReturn ret = node->Hook(this, BT::HookActionType::GET);
        if(ret.isValid())
        {
            /* truth table for logic --> XOR
                | Buf | Inv |  SUCCESS  |
                |  0  |  0  |   False   |
                |  0  |  1  |   True    |
                |  1  |  0  |   True    |
                |  1  |  1  |   False   |*/
             ///[fgn] apply inverted flag
            if (ret.getVal() ^ this->useInvertedLogic_)
                status = BT::ReturnStatus::SUCCESS;
            else
                status = BT::ReturnStatus::FAILURE;
            /// only check once, if one hook returned a valid value we leave it at that
            break;
        }
    }
    this->set_status(status);
    return status;
}
