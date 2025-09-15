#ifndef SO_ARM101_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_
#define SO_ARM101_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SO_ARM101_ROS2_CONTROL_EXPORT __attribute__ ((dllexport))
    #define SO_ARM101_ROS2_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define SO_ARM101_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define SO_ARM101_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef SO_ARM101_ROS2_CONTROL_BUILDING_LIBRARY
    #define SO_ARM101_ROS2_CONTROL_PUBLIC SO_ARM101_ROS2_CONTROL_EXPORT
  #else
    #define SO_ARM101_ROS2_CONTROL_PUBLIC SO_ARM101_ROS2_CONTROL_IMPORT
  #endif
  #define SO_ARM101_ROS2_CONTROL_PUBLIC_TYPE SO_ARM101_ROS2_CONTROL_PUBLIC
  #define SO_ARM101_ROS2_CONTROL_LOCAL
#else
  #define SO_ARM101_ROS2_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define SO_ARM101_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define SO_ARM101_ROS2_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define SO_ARM101_ROS2_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SO_ARM101_ROS2_CONTROL_PUBLIC
    #define SO_ARM101_ROS2_CONTROL_LOCAL
  #endif
  #define SO_ARM101_ROS2_CONTROL_PUBLIC_TYPE
#endif

#endif  // SO_ARM101_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_