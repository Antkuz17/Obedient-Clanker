/* Minimal stand-in for QNX <camera/camera_api.h>, only for compiling and testing camgrab.c
 * on a laptop. Field and function names mirror the real API as used by camgrab.c. */
#pragma once
#include <stdint.h>
#include <errno.h>
#ifndef EOK
#define EOK 0
#endif
typedef int camera_handle_t;
typedef int camera_unit_t;
typedef int camera_error_t;
#define CAMERA_HANDLE_INVALID (-1)
#define CAMERA_MODE_RO 1
typedef enum { CAMERA_FRAMETYPE_UNSPECIFIED = 0, CAMERA_FRAMETYPE_NV12 = 1,
               CAMERA_FRAMETYPE_YCBYCR = 2 } camera_frametype_t;
typedef enum { CAMERA_IMGPROP_FORMAT = 1, CAMERA_IMGPROP_CREATEWINDOW, CAMERA_IMGPROP_WIDTH,
               CAMERA_IMGPROP_HEIGHT } camera_imgprop_t;
typedef struct { uint32_t height, width, stride; int64_t uv_offset; int64_t uv_stride; } camera_frame_nv12_t;
typedef struct { uint32_t height, width, stride; } camera_frame_ycbycr_t;
typedef union { camera_frame_nv12_t nv12; camera_frame_ycbycr_t ycbycr; } camera_frame_desc_t;
typedef struct { camera_frametype_t frametype; uint64_t framesize; uint8_t *framebuf;
                 camera_frame_desc_t framedesc; } camera_buffer_t;
typedef struct { unsigned width, height; } camera_res_t;
camera_error_t camera_open(camera_unit_t, uint32_t, camera_handle_t *);
camera_error_t camera_close(camera_handle_t);
camera_error_t camera_get_supported_cameras(uint32_t, uint32_t *, camera_unit_t *);
camera_error_t camera_get_supported_vf_frame_types(camera_handle_t, uint32_t, uint32_t *, camera_frametype_t *);
camera_error_t camera_get_supported_vf_resolutions(camera_handle_t, uint32_t, uint32_t *, camera_res_t *);
camera_error_t camera_set_vf_property(camera_handle_t, ...);
camera_error_t camera_get_vf_property(camera_handle_t, ...);
camera_error_t camera_start_viewfinder(camera_handle_t,
    void (*)(camera_handle_t, camera_buffer_t *, void *), void (*)(camera_handle_t, int, uint16_t, void *), void *);
camera_error_t camera_stop_viewfinder(camera_handle_t);
