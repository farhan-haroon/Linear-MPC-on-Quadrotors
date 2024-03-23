#ifndef TEST_SIM__VISIBILITY_CONTROL_H_
#define TEST_SIM__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TEST_SIM_EXPORT __attribute__ ((dllexport))
    #define TEST_SIM_IMPORT __attribute__ ((dllimport))
  #else
    #define TEST_SIM_EXPORT __declspec(dllexport)
    #define TEST_SIM_IMPORT __declspec(dllimport)
  #endif
  #ifdef TEST_SIM_BUILDING_LIBRARY
    #define TEST_SIM_PUBLIC TEST_SIM_EXPORT
  #else
    #define TEST_SIM_PUBLIC TEST_SIM_IMPORT
  #endif
  #define TEST_SIM_PUBLIC_TYPE TEST_SIM_PUBLIC
  #define TEST_SIM_LOCAL
#else
  #define TEST_SIM_EXPORT __attribute__ ((visibility("default")))
  #define TEST_SIM_IMPORT
  #if __GNUC__ >= 4
    #define TEST_SIM_PUBLIC __attribute__ ((visibility("default")))
    #define TEST_SIM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TEST_SIM_PUBLIC
    #define TEST_SIM_LOCAL
  #endif
  #define TEST_SIM_PUBLIC_TYPE
#endif
#endif  // TEST_SIM__VISIBILITY_CONTROL_H_
// Generated 06-Feb-2024 19:42:35
// Copyright 2019-2020 The MathWorks, Inc.
