#ifndef MY_SIM__VISIBILITY_CONTROL_H_
#define MY_SIM__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MY_SIM_EXPORT __attribute__ ((dllexport))
    #define MY_SIM_IMPORT __attribute__ ((dllimport))
  #else
    #define MY_SIM_EXPORT __declspec(dllexport)
    #define MY_SIM_IMPORT __declspec(dllimport)
  #endif
  #ifdef MY_SIM_BUILDING_LIBRARY
    #define MY_SIM_PUBLIC MY_SIM_EXPORT
  #else
    #define MY_SIM_PUBLIC MY_SIM_IMPORT
  #endif
  #define MY_SIM_PUBLIC_TYPE MY_SIM_PUBLIC
  #define MY_SIM_LOCAL
#else
  #define MY_SIM_EXPORT __attribute__ ((visibility("default")))
  #define MY_SIM_IMPORT
  #if __GNUC__ >= 4
    #define MY_SIM_PUBLIC __attribute__ ((visibility("default")))
    #define MY_SIM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MY_SIM_PUBLIC
    #define MY_SIM_LOCAL
  #endif
  #define MY_SIM_PUBLIC_TYPE
#endif
#endif  // MY_SIM__VISIBILITY_CONTROL_H_
// Generated 08-Feb-2024 23:43:59
// Copyright 2019-2020 The MathWorks, Inc.
