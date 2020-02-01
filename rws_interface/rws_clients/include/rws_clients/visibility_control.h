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

#ifndef RWS_CLIENTS__VISIBILITY_CONTROL_H_
#define RWS_CLIENTS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RWS_CLIENTS_EXPORT __attribute__ ((dllexport))
    #define RWS_CLIENTS_IMPORT __attribute__ ((dllimport))
  #else
    #define RWS_CLIENTS_EXPORT __declspec(dllexport)
    #define RWS_CLIENTS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RWS_CLIENTS_BUILDING_DLL
    #define RWS_CLIENTS_PUBLIC RWS_CLIENTS_EXPORT
  #else
    #define RWS_CLIENTS_PUBLIC RWS_CLIENTS_IMPORT
  #endif
  #define RWS_CLIENTS_PUBLIC_TYPE RWS_CLIENTS_PUBLIC
  #define RWS_CLIENTS_LOCAL
#else
  #define RWS_CLIENTS_EXPORT __attribute__ ((visibility("default")))
  #define RWS_CLIENTS_IMPORT
  #if __GNUC__ >= 4
    #define RWS_CLIENTS_PUBLIC __attribute__ ((visibility("default")))
    #define RWS_CLIENTS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RWS_CLIENTS_PUBLIC
    #define RWS_CLIENTS_LOCAL
  #endif
  #define RWS_CLIENTS_PUBLIC_TYPE
#endif

#endif  // RWS_CLIENTS__VISIBILITY_CONTROL_H_