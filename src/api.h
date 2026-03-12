#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define BoxDemoController_DLLIMPORT __declspec(dllimport)
#  define BoxDemoController_DLLEXPORT __declspec(dllexport)
#  define BoxDemoController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define BoxDemoController_DLLIMPORT __attribute__((visibility("default")))
#    define BoxDemoController_DLLEXPORT __attribute__((visibility("default")))
#    define BoxDemoController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define BoxDemoController_DLLIMPORT
#    define BoxDemoController_DLLEXPORT
#    define BoxDemoController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef BoxDemoController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define BoxDemoController_DLLAPI
#  define BoxDemoController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef BoxDemoController_EXPORTS
#    define BoxDemoController_DLLAPI BoxDemoController_DLLEXPORT
#  else
#    define BoxDemoController_DLLAPI BoxDemoController_DLLIMPORT
#  endif // BoxDemoController_EXPORTS
#  define BoxDemoController_LOCAL BoxDemoController_DLLLOCAL
#endif // BoxDemoController_STATIC
