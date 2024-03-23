#ifndef ROUGH__VISIBILITY_CONTROL_H_
#define ROUGH__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROUGH_EXPORT __attribute__ ((dllexport))
    #define ROUGH_IMPORT __attribute__ ((dllimport))
  #else
    #define ROUGH_EXPORT __declspec(dllexport)
    #define ROUGH_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROUGH_BUILDING_LIBRARY
    #define ROUGH_PUBLIC ROUGH_EXPORT
  #else
    #define ROUGH_PUBLIC ROUGH_IMPORT
  #endif
  #define ROUGH_PUBLIC_TYPE ROUGH_PUBLIC
  #define ROUGH_LOCAL
#else
  #define ROUGH_EXPORT __attribute__ ((visibility("default")))
  #define ROUGH_IMPORT
  #if __GNUC__ >= 4
    #define ROUGH_PUBLIC __attribute__ ((visibility("default")))
    #define ROUGH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROUGH_PUBLIC
    #define ROUGH_LOCAL
  #endif
  #define ROUGH_PUBLIC_TYPE
#endif
#endif  // ROUGH__VISIBILITY_CONTROL_H_
// Generated 08-Feb-2024 02:21:13
// Copyright 2019-2020 The MathWorks, Inc.
