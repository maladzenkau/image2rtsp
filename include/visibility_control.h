#ifndef IMAGE2RTSP__VISIBILITY_CONTROL_H_
#define IMAGE2RTSP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE2RTSP_EXPORT __attribute__ ((dllexport))
    #define IMAGE2RTSP_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE2RTSP_EXPORT __declspec(dllexport)
    #define IMAGE2RTSP_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE2RTSP_BUILDING_DLL
    #define IMAGE2RTSP_PUBLIC IMAGE2RTSP_EXPORT
  #else
    #define IMAGE2RTSP_PUBLIC IMAGE2RTSP_IMPORT
  #endif
  #define IMAGE2RTSP_PUBLIC_TYPE IMAGE2RTSP_PUBLIC
  #define IMAGE2RTSP_LOCAL
#else
  #define IMAGE2RTSP_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE2RTSP_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE2RTSP_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE2RTSP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE2RTSP_PUBLIC
    #define IMAGE2RTSP_LOCAL
  #endif
  #define IMAGE2RTSP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // IMAGE2RTSP__VISIBILITY_CONTROL_H_