#ifndef SPIN_CONTROLLER__VISIBILITY_CONTROL_H_
#define SPIN_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SPIN_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define SPIN_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define SPIN_CONTROLLER_EXPORT __declspec(dllexport)
    #define SPIN_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef SPIN_CONTROLLER_BUILDING_LIBRARY
    #define SPIN_CONTROLLER_PUBLIC SPIN_CONTROLLER_EXPORT
  #else
    #define SPIN_CONTROLLER_PUBLIC SPIN_CONTROLLER_IMPORT
  #endif
  #define SPIN_CONTROLLER_PUBLIC_TYPE SPIN_CONTROLLER_PUBLIC
  #define SPIN_CONTROLLER_LOCAL
#else
  #define SPIN_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define SPIN_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define SPIN_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define SPIN_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SPIN_CONTROLLER_PUBLIC
    #define SPIN_CONTROLLER_LOCAL
  #endif
  #define SPIN_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // SPIN_CONTROLLER__VISIBILITY_CONTROL_H_
