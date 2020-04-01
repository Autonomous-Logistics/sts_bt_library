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

#pragma once

#include <sts_bt_library/dot_bt.h>
#include <sts_bt_library/logger.h>

/// bt node types
#include <sts_bt_library/parallel_node.h>
#include <sts_bt_library/fallback_node.h>
#include <sts_bt_library/sequence_node.h>
#include <sts_bt_library/sequence_node_with_memory.h>
#include <sts_bt_library/fallback_node_with_memory.h>
/// other bt types
#include <sts_bt_library/exceptions.h>
///[fgn] extra types for me
#include <sts_bt_library/false_node.h>
#include <sts_bt_library/set_node.h>
#include <sts_bt_library/reset_node.h>
#include <sts_bt_library/flip_flop_node.h>
#include <sts_bt_library/condition_node_hook.h>
/// ...


#include <string>
#include <map>

#include <typeinfo>
#include <math.h>       /* pow */

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_srvs/Trigger.h"

#include <std_msgs/String.h>

#include <sts_bt_library/graphml/parser.h>

namespace sts_behavior_tree{



class BehaviorTree
{
public:
    /// bt worker functions
    static void setLogLevel(int log_level);
    static void init(BT::TreeNode* root, int TickPeriod_milliseconds, ros::NodeHandle* nh);
    static void Execute(BT::TreeNode* root, int TickPeriod_milliseconds, std::string& id, ros::NodeHandle* nh, std::string nodeName, bool events, bool feedback, int debug_root_delay_);
    static void syncInputBuffers(BT::TreeNode* root);

    ///bt handling functions
    static void handleEvent(BT::Event e);

    /// bt parsing functions
    static BT::TreeNode* Load(std::string graphmlPath, ros::NodeHandle* nh);
    static BT::TreeNode* createBTNodes(graphml::Config &parsedCfg, std::vector<BT::TreeNode*> &nodes, ros::NodeHandle* nh);
    static void connectBTNodes(graphml::Config &treeCfg, std::vector<BT::TreeNode*> &nodes);
    static BT::TreeNode* findNodeById(std::string& id, std::vector<BT::TreeNode*> &nodes);

    /// dotBt functions
    static void statusChangedEvent(BT::TreeNode* node, BT::ReturnStatus status);
    static bool ReLoad(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

public:
    /// some for singleton behavior (bt is not a class [which it should be] so we do it this way for now)
    static BT::DotBt* dotBtPtr;
    static BT::Logger* loggerPtr;

    static std::string lastGraphmlPath;
    static ros::NodeHandle* lastNhPtr;
    static BT::TreeNode* lastRootPtr;
    static unsigned int logLevel;
    static int lastTickPeriod;
    static std::mutex dataAccessMutex_;
    static ros::Publisher eventPub_;
    static ros::Publisher eventStrPub_;


};


} //namespace sts_behavior_tree
