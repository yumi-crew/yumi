#ifndef REFERENCE_SETTER__VISIBILITY_CONTROL_H_
#define REFERENCE_SETTER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REFERENCE_SETTER_EXPORT __attribute__ ((dllexport))
    #define REFERENCE_SETTER_IMPORT __attribute__ ((dllimport))
  #else
    #define REFERENCE_SETTER_EXPORT __declspec(dllexport)
    #define REFERENCE_SETTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef REFERENCE_SETTER_BUILDING_DLL
    #define REFERENCE_SETTER_PUBLIC REFERENCE_SETTER_EXPORT
  #else
    #define REFERENCE_SETTER_PUBLIC REFERENCE_SETTER_IMPORT
  #endif
  #define REFERENCE_SETTER_PUBLIC_TYPE REFERENCE_SETTER_PUBLIC
  #define REFERENCE_SETTER_LOCAL
#else
  #define REFERENCE_SETTER_EXPORT __attribute__ ((visibility("default")))
  #define REFERENCE_SETTER_IMPORT
  #if __GNUC__ >= 4
    #define REFERENCE_SETTER_PUBLIC __attribute__ ((visibility("default")))
    #define REFERENCE_SETTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REFERENCE_SETTER_PUBLIC
    #define REFERENCE_SETTER_LOCAL
  #endif
  #define REFERENCE_SETTER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // REFERENCE_SETTER__VISIBILITY_CONTROL_H_