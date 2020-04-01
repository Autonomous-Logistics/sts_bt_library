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

#ifndef LOGGER_H
#define LOGGER_H


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sts_bt_library/bt_log.h>


#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>


namespace BT
{

class Logger
{
public:
  Logger(ros::NodeHandle* nhPtr, unsigned int logLevel);

  bool isPrintAllowed(unsigned int minLevel);

  void logRos(std::string msg);
  void flush();

private:
  void initialize();
  void clear();
  void prepareBuf(std::string& str);
  //void send(std::string str);
  void send();

public:

private:
  ros::NodeHandle* nh_;
  unsigned int logLevel_;
  std::vector<std::string> logBuf_;
  ros::Publisher logPub_;


};

}  // namespace BT
#endif  // LOGGER_H
