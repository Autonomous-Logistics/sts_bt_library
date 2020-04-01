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

#include <sts_bt_library/logger.h>

#include <string>
#include <std_msgs/String.h>
#include <cctype>
#include <algorithm>

namespace BT
{




BT::Logger::Logger(ros::NodeHandle* nhPtr, unsigned int logLevel){
    this->nh_ = nhPtr;
    this->logLevel_ = logLevel;
    this->initialize();
}

bool Logger::isPrintAllowed(unsigned int minLevel){
    bool allowed = false;
    if(this->logLevel_ >= minLevel )
        allowed = true;
    return allowed;
}

void Logger::initialize(){
    this->logPub_ = this->nh_->advertise<sts_bt_library::bt_log>("log", 1);
}

void BT::Logger::logRos(std::string msg){
    this->logBuf_.push_back(msg);
}

void Logger::flush(){
    //std::string str = "";
    //this->prepareBuf(str);
    // send our messages
    //this->send(str);
    this->send();
    // clear buffer
    this->clear();
}

void Logger::clear(){
    this->logBuf_.clear();
}

void Logger::prepareBuf(std::string& str){
    const char* const delim = "\n";
    std::ostringstream stream;
    std::copy(this->logBuf_.begin(), this->logBuf_.end(),
               std::ostream_iterator<std::string>(stream, delim));
    str = stream.str();
}

void Logger::send(){
    sts_bt_library::bt_log message;
    message.stamp = ros::Time::now();
    message.log = this->logBuf_;
    this->logPub_.publish(message);
}




}  // namespace BT
