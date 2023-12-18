#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>

void testIni() {
    using boost::property_tree::ptree;
    ptree pt;

    read_ini("data/config.ini", pt);

    for (const auto& section : pt) {
        std::cout << "[" << section.first << "]\n";
        for (const auto& key : section.second) {
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
        }
    }
    std::cout << std::endl;

    pt.put("Database.Port", "5432");
    write_ini("data/config.ini", pt);
}

void testXml() {
    using boost::property_tree::ptree;
    ptree pt;

    read_xml("data/undistort.launch", pt);

    for (auto& node : pt.get_child("launch")) {
        std::string nodeName = node.first;
        if (nodeName == "arg") {
            std::string argName = node.second.get("<xmlattr>.name", "");
            std::string argDefault = node.second.get("<xmlattr>.default", "");
            std::cout << "Arg name: " << argName << ", default: " << argDefault << std::endl;
        } else if (nodeName == "node") {
            std::string nodeName = node.second.get("<xmlattr>.name", "");
            std::string nodePkg = node.second.get("<xmlattr>.pkg", "");
            std::string nodeType = node.second.get("<xmlattr>.type", "");
            std::cout << "\nNode name: " << nodeName << ", pkg: " << nodePkg << ", type: " << nodeType << std::endl;
            for (auto& param : node.second) {
                if (param.first == "param") {
                    std::string paramName = param.second.get("<xmlattr>.name", "");
                    std::string paramValue = param.second.get("<xmlattr>.value", "");
                    std::cout << "Param name: " << paramName << ", value: " << paramValue << std::endl;
                } else if (param.first == "remap") {
                    std::string remapFrom = param.second.get("<xmlattr>.from", "");
                    std::string remapTo = param.second.get("<xmlattr>.to", "");
                    std::cout << "Remap from: " << remapFrom << ", to: " << remapTo << std::endl;
                } else if (param.first == "rosparam") {
                    std::string rosparamCommand = param.second.get("<xmlattr>.command", "");
                    std::string rosparamFile = param.second.get("<xmlattr>.file", "");
                    std::cout << "Rosparam command: " << rosparamCommand << ", file: " << rosparamFile << std::endl;
                }
            }
        }
    }
    std::cout << std::endl;
}

int main() {
    testIni();
    testXml();

    return 0;
}