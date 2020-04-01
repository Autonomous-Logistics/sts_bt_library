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

#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H


#include <sts_bt_library/tree_node.h>
#include <vector>
#include <string>

namespace BT
{
class ControlNode : public TreeNode
{
public:
    // Constructor
    explicit ControlNode(std::string id, std::string name, BT::Logger* logger);
    virtual ~ControlNode();

    /// must have virtual override functions
    virtual BT::ReturnStatus Tick(std::string& id) = 0;
    virtual int DrawType() = 0;
    virtual void Sync();
    virtual void Halt();
    virtual void ResetColorState();
    virtual int Depth();

    /// own functions
protected:
    void HaltChildren(std::size_t i);
    bool WriteState(ReturnStatus new_state);
    /// child i status. Used to route the ticks
    ReturnStatus child_i_status_;
};
}  // namespace BT

#endif  // CONTROL_NODE_H
