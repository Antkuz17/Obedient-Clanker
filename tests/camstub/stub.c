/* Fake libcamapi: 640x480 frames with stride padding (stride = width + 64). The pixel at
 * (x, y) of frame k has Y = (x + y + k) & 0xff, so the reader can check rows and ordering.
 * CAMSTUB_FMT=yuyv selects YCbYCr, otherwise NV12. */
#include "camera/camera_api.h"
#include <pthread.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define W 640
#define H 480
#define PAD 64
static void (*cb)(camera_handle_t, camera_buffer_t *, void *);
static volatile int running;
static int yuyv(void) { const char *f = getenv("CAMSTUB_FMT"); return f && !strcmp(f, "yuyv"); }
camera_error_t camera_open(camera_unit_t u, uint32_t m, camera_handle_t *h) { (void)u; (void)m; *h = 1; return EOK; }
camera_error_t camera_close(camera_handle_t h) { (void)h; return EOK; }
camera_error_t camera_get_supported_cameras(uint32_t a, uint32_t *n, camera_unit_t *u) { (void)a; *n = 1; u[0] = 5; return EOK; }
camera_error_t camera_get_supported_vf_frame_types(camera_handle_t h, uint32_t a, uint32_t *n, camera_frametype_t *t) {
    (void)h; (void)a; *n = 1; t[0] = yuyv() ? CAMERA_FRAMETYPE_YCBYCR : CAMERA_FRAMETYPE_NV12; return EOK; }
camera_error_t camera_get_supported_vf_resolutions(camera_handle_t h, uint32_t a, uint32_t *n, camera_res_t *r) {
    (void)h; (void)a; *n = 1; r[0].width = W; r[0].height = H; return EOK; }
camera_error_t camera_set_vf_property(camera_handle_t h, ...) { (void)h; return EOK; }
camera_error_t camera_get_vf_property(camera_handle_t h, ...) {
    va_list ap; va_start(ap, h);
    for (int i = 0; i < 2; i++) { int prop = va_arg(ap, int); int *p = va_arg(ap, int *);
        *p = prop == CAMERA_IMGPROP_WIDTH ? W : H; }
    va_end(ap); return EOK; }
static void *run(void *arg) {
    (void)arg; int bpp = yuyv() ? 2 : 1, stride = W * bpp + PAD;
    uint8_t *buf = malloc((size_t)stride * H * 2);
    for (unsigned k = 0; running; k++) {
        camera_buffer_t b; memset(&b, 0, sizeof b); b.framebuf = buf;
        memset(buf, 0xEE, (size_t)stride * H * 2);              /* padding = 0xEE junk */
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++)
            buf[(size_t)y * stride + x * bpp] = (uint8_t)(x + y + k);
        if (yuyv()) { b.frametype = CAMERA_FRAMETYPE_YCBYCR;
            for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) buf[(size_t)y * stride + x * 2 + 1] = 128;
            b.framedesc.ycbycr.width = W; b.framedesc.ycbycr.height = H; b.framedesc.ycbycr.stride = stride; }
        else { b.frametype = CAMERA_FRAMETYPE_NV12; size_t uvo = (size_t)stride * H;
            for (int y = 0; y < H / 2; y++) memset(buf + uvo + (size_t)y * stride, 128, W);
            b.framedesc.nv12 = (camera_frame_nv12_t){H, W, stride, (int64_t)uvo, stride}; }
        cb(1, &b, NULL); usleep(33000);
    }
    free(buf); return NULL;
}
static pthread_t th;
camera_error_t camera_start_viewfinder(camera_handle_t h, void (*f)(camera_handle_t, camera_buffer_t *, void *),
    void (*s)(camera_handle_t, int, uint16_t, void *), void *a) {
    (void)h; (void)s; (void)a; cb = f; running = 1; pthread_create(&th, NULL, run, NULL); return EOK; }
camera_error_t camera_stop_viewfinder(camera_handle_t h) { (void)h; running = 0; pthread_join(th, NULL); return EOK; }
