//
// Created by pohzhiee on 22/10/19.
//

#include "parameter_server/yaml_parser.hpp"

#include <memory>
#include <string>
#include <unordered_map>

#include "parameter_server/yaml_parser.hpp"

// It is unfortunate that we are using gcc 7.4 which doesn't have filesystem fully implemented yet, only experimental
// Remove the experimental once we move past gcc 7.4
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

namespace parameter_server {

    namespace {

        constexpr char separator = '.';

        void get_key_values(
                YAML::Node root, const std::string& key, std::unordered_map<std::string, std::string> &key_values) {
            if (root.Type() == YAML::NodeType::Scalar) {
                key_values.emplace(key, root.as<std::string>());
                return;
            }

            if (root.Type() == YAML::NodeType::Map) {
                for (auto root_it = root.begin(); root_it != root.end(); ++root_it) {
                    get_key_values(
                            root_it->second, key + separator + root_it->first.as<std::string>(), key_values);
                }
            }

            if (root.Type() == YAML::NodeType::Sequence) {
                size_t index = 0;
                for (auto root_it = root.begin(); root_it != root.end(); ++root_it) {
                    get_key_values(*root_it, key + separator + std::to_string(index++), key_values);
                }
            }
        }

    }  // namespace

    void
    YamlParser::parse(const std::string &absolute_file_path) {
        if (!fs::path(absolute_file_path).is_absolute()) {
            throw std::runtime_error(std::string("no absolute file path given: ") + absolute_file_path);
        }

        root_node_ = YAML::LoadFile(absolute_file_path);
    }

    std::unordered_map<std::string, std::string>
    YamlParser::get_key_value_pairs() {
        std::unordered_map<std::string, std::string> key_values;
        get_key_values(root_node_, "", key_values);
        return key_values;
    }

}  // namespace parameter_server
