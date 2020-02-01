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

#ifndef SG_CONTROL__VISIBILITY_CONTROL_H_
#define SG_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SG_CONTROL_EXPORT __attribute__ ((dllexport))
    #define SG_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define SG_CONTROL_EXPORT __declspec(dllexport)
    #define SG_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef SG_CONTROL_BUILDING_DLL
    #define SG_CONTROL_PUBLIC SG_CONTROL_EXPORT
  #else
    #define SG_CONTROL_PUBLIC SG_CONTROL_IMPORT
  #endif
  #define SG_CONTROL_PUBLIC_TYPE SG_CONTROL_PUBLIC
  #define SG_CONTROL_LOCAL
#else
  #define SG_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define SG_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define SG_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define SG_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SG_CONTROL_PUBLIC
    #define SG_CONTROL_LOCAL
  #endif
  #define SG_CONTROL_PUBLIC_TYPE
#endif

#endif  // SG_CONTROL__VISIBILITY_CONTROL_H_