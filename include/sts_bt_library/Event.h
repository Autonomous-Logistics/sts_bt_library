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

#include <iostream>
#include <string>

#include <sts_bt_library/tree_node.h>
#include <sts_bt_library/bt_event.h>

#include <ros/time.h>

namespace BT
{
class Event
{

public:
    Event(int id, BT::TreeNode* sender, std::string msg){
        this->stamp_ = ros::Time::now(); this->sender_ = sender; this->msg_ = msg; this->id_ = id;
    }
    std::string toString(){std::ostringstream os;os<<this;return os.str();}
    friend std::ostream& operator<<(std::ostream &out, Event* e) {
        return out << "[" << e->stamp_.nsec << "]" << e->sender_->get_name()<< "<"<<e->id_<<">" << ": " << e->msg_;
    }
    sts_bt_library::bt_event toRosMsg(){
        sts_bt_library::bt_event x; x.stamp = this->stamp_; x.event_id = this->id_; x.sender = this->sender_->get_name(); x.msg=this->msg_;
        return x;
    }
    ros::Time stamp_;
    BT::TreeNode* sender_;
    std::string msg_;
    int id_;
};

}
