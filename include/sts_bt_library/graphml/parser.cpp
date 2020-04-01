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

#include <iostream>

#include "parser.h"

namespace graphml
{

Config parse(std::string path)
{
    //std::cout << "Starting ... \n";
    Config cfg;

    tinyxml2::XMLDocument doc;
    doc.LoadFile(path.c_str());
    if(doc.Error())
    {
        std::cout << "Err: "<< doc.Error() << "\n" << doc.ErrorName() <<"\n"<< doc.ErrorStr();
        return cfg;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("graphml");
    if(root != NULL)
    {
        /// find necessary configuration keys
        //std::cout << "Checking keys ...\n";
        tinyxml2::XMLElement* key=root->FirstChildElement("key");
        for(;key != NULL; key=key->NextSiblingElement("key"))
            cfg.checkKeyElement(key);

        /// crawl through graph and set up tree
        //std::cout << "Creating Nodes ...\n";
        tinyxml2::XMLElement* node=root->FirstChildElement("graph")->FirstChildElement("node");
        for(;node != NULL; node=node->NextSiblingElement("node"))
        {
            cfg.checkNodeElement(node);
        }

        /// check for edges
        //std::cout << "Checking Connections ...\n";
        tinyxml2::XMLElement* edge=root->FirstChildElement("graph")->FirstChildElement("edge");
        for(;edge != NULL; edge=edge->NextSiblingElement("edge"))
            cfg.checkEdgeElement(edge);

        /// apply those edges and order them if needed
        ///if we are a hook, we want to add the connection to special hooks list
        /// else it is a normal connection between two relevant nodes which will be added to children list
        for(auto edge : cfg.edges)
        {
            if(edge.isHook == true)
                edge.source->hooks.push_back(edge.target);
            else
                edge.source->children.push_back(edge.target);
        }
        ///sort all children based on parents horizontal allignment
        for(BTNode &node : cfg.nodeList)
        {
            std::sort(node.children.begin(), node.children.end(), BTNode());
        }
        /// cfg object now has a list of all the relevant nodes which we could iterate
        /*for(auto node: cfg.nodeList)
        { std::cout << node;}*/
    }

    std::cout << "DONE ... ";
    return cfg;
}

}//namespace graphml


