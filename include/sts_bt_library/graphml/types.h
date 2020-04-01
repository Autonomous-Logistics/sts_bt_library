/* Copyright (C) 2019-2020 Sebastian Mueller, Nick Fiege - All Rights Reserved
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

#include <string>
#include <iostream>
#include <algorithm>
#include <unordered_map>

#include "tinyxml2.h"
#include <vector>

namespace graphml
{


class BTNode
{
public:
    enum class BT_TYPE{NONE, ROOT, SEQUENCE, FALLBACK, PARALLEL, DECORATOR,   SEQUENCE_MEMORY, FALLBACK_MEMORY,      CONDITION, ACTION,     FUNCTION};
    static std::unordered_map<std::string, BT_TYPE> BT_TYPE_MAP;
    enum class BT_FUNCTION{NONE, TOPIC, SERVICE, ACTION,      FLIPFLOP, SET, RESET       , HOOK   , FALSE};
    static std::unordered_map<std::string, BT_FUNCTION> BT_FUNCTION_MAP;
    enum class BT_CONNECTION{NONE, HOOK};
    static std::unordered_map<std::string, BT_CONNECTION> BT_CONNECTION_MAP;

    void setTypeWithString(std::string typeStr);
    void setFunctionWithString(std::string functionStr);

    void setText(std::string text);
    void setPos(double x, double y);

    std::string getTypeString();
    std::string getFunctionString();

    bool isValid();

    inline friend std::ostream& operator<<(std::ostream & _stream, BTNode & node) {
            _stream << "BTNode["<<node.id<<"]: " << node.getTypeString() <<", " << node.getFunctionString() << " > " << node.text << std::endl;
            for(auto child : node.children)
                _stream << " -- " << child->id << ": " << child->text << std::endl;
            for(auto hook : node.hooks)
                _stream << " -} " << hook->id << std::endl;
            return _stream;
        }


    BT_TYPE type = BT_TYPE::NONE;
    BT_FUNCTION function = BT_FUNCTION::NONE;
    std::string content = "";
    std::string id = "";
    std::string text = "";
    double x; double y;
    std::vector<BTNode*> children;
    std::vector<BTNode*> hooks;

    /// sorting operator, sort by lowest x left
    bool operator() (const BTNode* l, const BTNode* r) const { return l->x < r->x; }
};

class BTEdge
{
public:
    BTEdge(BTNode* source, BTNode* target)
    {this->source = source; this->target = target;}

    BTNode* source;
    BTNode* target;
    bool isHook = false;
};


class Config
{
public:
    static const std::string BT_ATTRIBUTE;
    static const std::string BT_TYPE_KEY;
    static const std::string BT_FUNCTION_KEY;
    static const std::string BT_CONNECTION_KEY;

    static const std::string YED_NODEGRAHPICS_ATTRIBUTE;
    static const std::string YED_NODEGRAHPICS_KEY ;

    std::string type_id = "";
    std::string function_id = "";
    std::string connection_id = "";
    std::string nodeGraphics_id = "";

    std::vector<BTNode> nodeList;
    std::vector<BTEdge> edges;

    bool checkKeyElement(tinyxml2::XMLElement* key);
    bool checkNodeElement(tinyxml2::XMLElement* node);
    bool checkEdgeElement(tinyxml2::XMLElement* edge);

    bool checkKey(std::string keyVal, std::string idVal);

    BTNode* findNodeById(std::string id);

};


}
