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

#include "types.h"

namespace graphml {

/// ###########################################################################
///                          G E N E R A L
/// ###########################################################################

/// STS KEYS
const std::string Config::BT_ATTRIBUTE= "attr.name";
const std::string Config::BT_TYPE_KEY = "bt_type";
const std::string Config::BT_FUNCTION_KEY = "bt_function";
const std::string Config::BT_CONNECTION_KEY = "bt_connection";
;
/// YED DEFAULT KEYS
/// ... <key for="node" id="d8" yfiles.type="nodegraphics"/>
const std::string Config::YED_NODEGRAHPICS_ATTRIBUTE = "yfiles.type";
const std::string Config::YED_NODEGRAHPICS_KEY = "nodegraphics";




std::unordered_map<std::string, BTNode::BT_TYPE> BTNode::BT_TYPE_MAP =
{   {"none", BTNode::BT_TYPE::NONE}, {"root", BTNode::BT_TYPE::ROOT},
    {"sequence", BTNode::BT_TYPE::SEQUENCE}, {"fallback", BTNode::BT_TYPE::FALLBACK}, {"decorator", BTNode::BT_TYPE::DECORATOR},
    {"sequencememory", BTNode::BT_TYPE::SEQUENCE_MEMORY}, {"fallbackmemory", BTNode::BT_TYPE::FALLBACK_MEMORY},
    {"parallel", BTNode::BT_TYPE::PARALLEL},

    {"condition", BTNode::BT_TYPE::CONDITION}, {"action", BTNode::BT_TYPE::ACTION},
    {"function", BTNode::BT_TYPE::FUNCTION},
};
std::unordered_map<std::string, BTNode::BT_FUNCTION> BTNode::BT_FUNCTION_MAP =
{   {"none", BTNode::BT_FUNCTION::NONE},
    {"topic", BTNode::BT_FUNCTION::TOPIC}, {"service", BTNode::BT_FUNCTION::SERVICE}, {"action", BTNode::BT_FUNCTION::ACTION},
    {"false", BTNode::BT_FUNCTION::FALSE},
    {"flipflop", BTNode::BT_FUNCTION::FLIPFLOP}, {"set", BTNode::BT_FUNCTION::SET}, {"reset", BTNode::BT_FUNCTION::RESET},
    {"hook", BTNode::BT_FUNCTION::HOOK},

};
std::unordered_map<std::string, BTNode::BT_CONNECTION> BTNode::BT_CONNECTION_MAP =
{   {"none", BTNode::BT_CONNECTION::NONE}, {"hook", BTNode::BT_CONNECTION::HOOK},
};



/// ###########################################################################
///                          N O D E
/// ###########################################################################
void BTNode::setTypeWithString(std::string str)
{   ///lowercase the str
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    ///map onto all the types
    this->type = BTNode::BT_TYPE_MAP[str];
}
void BTNode::setFunctionWithString(std::string str)
{   ///lowercase the str
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    ///map onto all the types
    if(BT_FUNCTION_MAP.count(str) > 0) this->function = BTNode::BT_FUNCTION_MAP[str];
    else this->function = BTNode::BT_FUNCTION::NONE;
}
void BTNode::setText(std::string text){ this->text = text; }
void BTNode::setPos(double x, double y){this->x = x; this->y = y;}


std::string BTNode::getTypeString()
{
    for (auto it = BTNode::BT_TYPE_MAP.begin(); it != BTNode::BT_TYPE_MAP.end(); ++it)
        if (it->second == this->type){return it->first;}
    return "";
}
std::string BTNode::getFunctionString()
{
    for (auto it = BTNode::BT_FUNCTION_MAP.begin(); it != BTNode::BT_FUNCTION_MAP.end(); ++it)
        if (it->second == this->function){return it->first;}
    return "";
}
bool BTNode::isValid()
{
    return this->type != BTNode::BT_TYPE::NONE;
}

/// ###########################################################################
///                           C O N F I G
/// ###########################################################################
bool Config::checkKeyElement(tinyxml2::XMLElement* key)
{
    bool success = false;
    const char* attr = NULL;

    /// grab relevant attribute if possible
    if(attr == NULL)
        attr = key->Attribute(Config::BT_ATTRIBUTE.c_str());
    if(attr == NULL)
        attr = key->Attribute(Config::YED_NODEGRAHPICS_ATTRIBUTE.c_str());

    if(attr != NULL)
    {
        /// grab id
        const char* id = key->Attribute("id");
        if(id != NULL)
        {
            std::string keyVal = std::string(attr);
            std::string idVal = std::string(id);
            success = this->checkKey(keyVal, idVal);
        }
    }
    return success;
}


bool Config::checkNodeElement(tinyxml2::XMLElement* node)
{
    bool success = false;
    /// create node as a container to fill it with upcoming values
    BTNode btnode;
    /// grab node id tag
    std::string id = std::string(node->Attribute("id"));
    btnode.id = id;

    ///every node with a data field containing our extraced ids (example d4, d5) is valid
    tinyxml2::XMLElement* data=node->FirstChildElement("data");
    for(;data != NULL; data=data->NextSiblingElement("data"))
    {
        const char* key = data->Attribute("key");
        if(key != NULL)
        {
            std::string keyStr = std::string(key);
            //std::cout << "keyString: " << keyStr << "\n";
            if(keyStr != "")
            {
                tinyxml2::XMLNode* contentNode = data->FirstChild();
                if(contentNode != NULL)
                {
                    std::string content = contentNode->Value();
                    if(keyStr.compare(this->type_id) == 0)
                        btnode.setTypeWithString(content);
                    else if(keyStr.compare(this->function_id) == 0)
                        btnode.setFunctionWithString(content);
                    else if(keyStr.compare(this->nodeGraphics_id) == 0)
                    {
                        ///try to gather node label data
                        /// <data key="d8">
                        ///     <y:ShapeNode>
                        ///         <y:Geometry height="50.0" width="150.0" x="3534.269362001729" y="2586.810392929158"/>
                        ///         <y:Fill color="#E8EEF7" color2="#8775E6" transparent="false"/>
                        ///         <y:BorderStyle color="#000000" raised="false" type="line" width="1.0"/>
                        ///         <y:NodeLabel alignment="center" autoSizePolicy="content" fontFamily="Dialog" fontSize="14" fontStyle="plain" hasBackgroundColor="false" hasLineColor="false" height="20.296875" horizontalTextPosition="center" iconTextGap="4" modelName="custom" textColor="#000000" verticalTextPosition="bottom" visible="true" width="131.845703125" x="9.0771484375" y="14.8515625">someSrvCondition<y:LabelModel>
                        ///             <y:SmartNodeLabelModel distance="4.0"/>

                        ///parse node position in graph
                        tinyxml2::XMLElement* nodeGeometry = data->FirstChildElement()->FirstChildElement("y:Geometry");
                        const char* xPosAttr = nodeGeometry->Attribute("x");
                        const char* yPosAttr = nodeGeometry->Attribute("y");
                        if(xPosAttr != NULL && yPosAttr != NULL)
                        {
                            std::string xStr (xPosAttr);
                            std::string yStr (yPosAttr);
                            std::string::size_type sz;
                            double xPos = std::stod(xStr,&sz);
                            double yPos = std::stod(yStr,&sz);
                            btnode.setPos(xPos, yPos);
                        }

                        ///parse nodeLabel and set it to node
                        tinyxml2::XMLElement* nodeLabel = data->FirstChildElement()->FirstChildElement("y:NodeLabel");

                        const char* text = nodeLabel->GetText();
                        if (text != NULL)
                        {
                            std::string nodeText = std::string(text);
                            //std::cout << nodeLabel->GetText() << std::endl;
                            btnode.setText(nodeText);
                        }
                    }
                    else{}
                }


            }
        }
    }
    /// check if the created node is valid and add it to list if true
    if(btnode.isValid())
    {
       // std::cout << "Found valid node: " << btnode.getTypeString() << std::endl;
       this->nodeList.push_back(btnode);
    }
    return success;
}

bool Config::checkEdgeElement(tinyxml2::XMLElement* edge)
{
    bool success = false;
    const char* source = edge->Attribute("source");
    const char* target = edge->Attribute("target");

    if(source != NULL && target != NULL)
    {
        BTNode* sourceNode = this->findNodeById(source);
        BTNode* targetNode = this->findNodeById(target);
        /// only add edges between relevant nodes
        if(targetNode != NULL && sourceNode != NULL)
        {
            BTEdge btEdgeObj(sourceNode, targetNode);

            ///default edge type is a connector between two nodes
            /// check if this is a special hook thingy
            /// <edge id="e16" source="n19" target="n14">
            ///     <data key="d10"><![CDATA[hook]]></data>
            tinyxml2::XMLElement* data=edge->FirstChildElement("data");
            for(;data != NULL; data=data->NextSiblingElement("data"))
            {
                const char* key = data->Attribute("key");
                if(key != NULL)
                {
                    std::string keyStr = std::string(key);
                    if(keyStr != "")
                    {
                        tinyxml2::XMLNode* contentNode = data->FirstChild();
                        if(contentNode != NULL)
                        {
                            /// we either found a hook or we didnt
                            std::string content = contentNode->Value();
                            if(keyStr.compare(this->connection_id) == 0)
                            {
                                btEdgeObj.isHook = true;
                                break;
                            }
                            /// hooks also are specified by a "dotted" line type in yed!!!!
                            /// this is hacky, but avoids typing datafields for every hook-line all the time
                            else
                            {
                                /// <data key="d14">
                                ///     <y:PolyLineEdge>
                                ///         <y:Path sx="0.0" sy="0.0" tx="0.0" ty="0.0"/>
                                ///         <y:LineStyle color="#00FF00" type="dotted" width="2.0"/>
                                ///         <y:Arrows source="none" target="circle"/>
                                ///         <y:BendStyle smoothed="true"/>
                                ///     </y:PolyLineEdge>
                                tinyxml2::XMLElement* polyLineEdge = data->FirstChildElement()->FirstChildElement("y:LineStyle");
                                if(polyLineEdge != NULL)
                                {
                                    const char* lineTypeAttr = polyLineEdge->Attribute("type");
                                    if(lineTypeAttr != NULL)
                                    {
                                        std::string lineTypeStr = std::string(lineTypeAttr);
                                        if(lineTypeStr.compare("dotted") == 0)
                                        {
                                            btEdgeObj.isHook = true;
                                            break;
                                        }
                                    } // end if linetypeAttr != NULL
                                } // end if polyLineEdge != NULL
                            }// end if else of finding hook edge via special data fields


                        } // end if data content != NULL
                    } // end if key != ""
                } // end if key = NULL
            } // end for data in edge

            success = true;
            this->edges.push_back(btEdgeObj);
        } // end if both target and source node exist in our nodelist [findNodeById]
    } // end if target and source attribute != NULL in edge
    return success;
}

BTNode* Config::findNodeById(std::string id)
{
    for(std::size_t i=0; i < this->nodeList.size(); i++)
    {
        BTNode* node = &this->nodeList[i];
        if(node->id.compare(id) == 0)
            return node;
    }
    return NULL;
}


bool Config::checkKey(std::string key, std::string id)
{
    bool success = true;
    if(key.compare(Config::BT_TYPE_KEY) == 0)
    {
        //std::cout << "Found " << Config::BT_TYPE_KEY << " with id " << id << std::endl;
        this->type_id = id;
    }
    else if(key.compare(Config::BT_FUNCTION_KEY) == 0)
    {
        //std::cout << "Found " << Config::BT_FUNCTION_KEY << " with id " << id << std::endl;
        this->function_id = id;
    }
    else if(key.compare(Config::BT_CONNECTION_KEY) == 0)
    {
        //std::cout << "Found " << Config::BT_CONNECTION_KEY << " with id " << id << std::endl;
        this->connection_id = id;
    }
    else if(key.compare(Config::YED_NODEGRAHPICS_KEY) == 0)
    {
        //std::cout << "Found " << Config::YED_NODEGRAHPICS_KEY << " with id " << id << std::endl;
        this->nodeGraphics_id = id;
    }
    else
    {
        success = false;
    }
    return success;
}




}//namespace graphml
