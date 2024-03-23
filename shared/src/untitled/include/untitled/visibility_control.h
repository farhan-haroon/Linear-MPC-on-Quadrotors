#ifndef UNTITLED__VISIBILITY_CONTROL_H_
#define UNTITLED__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UNTITLED_EXPORT __attribute__ ((dllexport))
    #define UNTITLED_IMPORT __attribute__ ((dllimport))
  #else
    #define UNTITLED_EXPORT __declspec(dllexport)
    #define UNTITLED_IMPORT __declspec(dllimport)
  #endif
  #ifdef UNTITLED_BUILDING_LIBRARY
    #define UNTITLED_PUBLIC UNTITLED_EXPORT
  #else
    #define UNTITLED_PUBLIC UNTITLED_IMPORT
  #endif
  #define UNTITLED_PUBLIC_TYPE UNTITLED_PUBLIC
  #define UNTITLED_LOCAL
#else
  #define UNTITLED_EXPORT __attribute__ ((visibility("default")))
  #define UNTITLED_IMPORT
  #if __GNUC__ >= 4
    #define UNTITLED_PUBLIC __attribute__ ((visibility("default")))
    #define UNTITLED_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UNTITLED_PUBLIC
    #define UNTITLED_LOCAL
  #endif
  #define UNTITLED_PUBLIC_TYPE
#endif
#endif  // UNTITLED__VISIBILITY_CONTROL_H_
// Generated 08-Feb-2024 01:34:35
// Copyright 2019-2020 The MathWorks, Inc.
