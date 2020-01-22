//
// Created by pohzhiee on 22/10/19.
//

#ifndef ROS2_CONTROL_PARAMETER_SERVER_YAML_PARSER_HPP
#define ROS2_CONTROL_PARAMETER_SERVER_YAML_PARSER_HPP\

#include <memory>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"

namespace parameter_server {
    class YamlParser {
    public:
        YamlParser() = default;

        virtual ~YamlParser() = default;

        void parse(const std::string &absolute_file_path);

        std::unordered_map<std::string, std::string>
        get_key_value_pairs();

    private:
        YAML::Node root_node_;
    };

} // namespace parameter_server

#endif //ROS2_CONTROL_PARAMETER_SERVER_YAML_PARSER_HPP
