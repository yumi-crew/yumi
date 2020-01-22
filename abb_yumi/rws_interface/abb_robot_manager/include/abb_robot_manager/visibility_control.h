// Copyright 2019 Norwegian University of Science and Technology.
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

#ifndef ABB_ROBOT_MANAGER__VISIBILITY_CONTROL_H_
#define ABB_ROBOT_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ABB_ROBOT_MANAGER_EXPORT __attribute__ ((dllexport))
    #define ABB_ROBOT_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define ABB_ROBOT_MANAGER_EXPORT __declspec(dllexport)
    #define ABB_ROBOT_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ABB_ROBOT_MANAGER_BUILDING_DLL
    #define ABB_ROBOT_MANAGER_PUBLIC ABB_ROBOT_MANAGER_EXPORT
  #else
    #define ABB_ROBOT_MANAGER_PUBLIC ABB_ROBOT_MANAGER_IMPORT
  #endif
  #define ABB_ROBOT_MANAGER_PUBLIC_TYPE ABB_ROBOT_MANAGER_PUBLIC
  #define ABB_ROBOT_MANAGER_LOCAL
#else
  #define ABB_ROBOT_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define ABB_ROBOT_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define ABB_ROBOT_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define ABB_ROBOT_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ABB_ROBOT_MANAGER_PUBLIC
    #define ABB_ROBOT_MANAGER_LOCAL
  #endif
  #define ABB_ROBOT_MANAGER_PUBLIC_TYPE
#endif

#endif  // ABB_ROBOT_MANAGER__VISIBILITY_CONTROL_H_