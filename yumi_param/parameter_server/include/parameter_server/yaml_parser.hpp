// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include "yaml-cpp/yaml.h"

namespace parameter_server 
{
    
class YamlParser 
{
public:
  YamlParser() = default;

  virtual ~YamlParser() = default;

  void parse(const std::string &absolute_file_path);

  std::unordered_map<std::string, std::string> get_key_value_pairs();

private:
  YAML::Node root_node_;
};

} // namespace parameter_server
