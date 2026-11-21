/*
 * camgrab: QNX camera (libcamapi) -> shared memory, for Python consumers.
 *
 * OpenCV can't open USB cameras on QNX (docs/qnx.md), so this tiny process owns the camera
 * and publishes the latest raw frame. Python reads it with edge/camera.py (ShmFrameReader).
 * Camera API usage follows QNX's own AI workshop (rpi_cam.cpp).
 *
 * Build on the Pi:   apk add qnx-sf-base-dev
 *                    cc -O2 -o camgrab camgrab.c -lcamapi
 * Run:               ./camgrab -u 5 -n camgrab_fwd      (unit 5 = first USB camera)
 *                    ./camgrab -l                       (list units, frame types, resolutions)
 *
 * Shared memory /<name> (QNX: /dev/shmem/<name>), little-endian, layout v1:
 *   off  type  field
 *   0    u32   magic 'CGRB' (0x42524743)
 *   4    u32   version (1)
 *   8    u32   width
 *   12   u32   height
 *   16   u32   format: 1 = YUYV (YCbYCr, 2 B/px), 2 = NV12 (Y plane + interleaved UV, 1.5 B/px)
 *   20   u32   frame_bytes (tightly packed, no stride padding)
 *   24   u64   seq: odd while writing, even when stable (seqlock); frame number = seq / 2
 *   32   u64   t_us: CLOCK_REALTIME at callback, microseconds (matches Python time.time())
 *   40   u32   writer pid
 *   44   ..    reserved, zero
 *   64   u8[]  frame
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <camera/camera_api.h>

#define MAGIC 0x42524743u
#define HDR 64
#define FMT_YUYV 1u
#define FMT_NV12 2u

typedef struct {
    uint32_t magic, version, width, height, format, frame_bytes;
    uint64_t seq, t_us;
    uint32_t pid;
} hdr_t;

static volatile sig_atomic_t g_stop = 0;
static uint8_t *g_map = NULL;
static uint32_t g_fmt = 0;

static void on_signal(int s) { (void)s; g_stop = 1; }

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* copy rows, dropping stride padding */
static void copy_plane(uint8_t *dst, const uint8_t *src, uint32_t row_bytes, uint32_t rows,
                       uint32_t stride) {
    if (stride == row_bytes) {
        memcpy(dst, src, (size_t)row_bytes * rows);
        return;
    }
    for (uint32_t r = 0; r < rows; r++) memcpy(dst + (size_t)r * row_bytes,
                                               src + (size_t)r * stride, row_bytes);
}

static void on_frame(camera_handle_t h, camera_buffer_t *buf, void *arg) {
    (void)h; (void)arg;
    hdr_t *hd = (hdr_t *)g_map;
    uint8_t *dst = g_map + HDR;
    if (buf->frametype != (g_fmt == FMT_NV12 ? CAMERA_FRAMETYPE_NV12
                                             : CAMERA_FRAMETYPE_YCBYCR)) return;

    __atomic_add_fetch(&hd->seq, 1, __ATOMIC_ACQ_REL);          /* odd: writing */
    if (g_fmt == FMT_NV12) {
        const camera_frame_nv12_t *d = &buf->framedesc.nv12;
        copy_plane(dst, buf->framebuf, d->width, d->height, d->stride);
        copy_plane(dst + (size_t)d->width * d->height, buf->framebuf + d->uv_offset,
                   d->width, d->height / 2, d->uv_stride);
    } else {
        const camera_frame_ycbycr_t *d = &buf->framedesc.ycbycr;
        copy_plane(dst, buf->framebuf, d->width * 2, d->height, d->stride);
    }
    hd->t_us = now_us();
    __atomic_add_fetch(&hd->seq, 1, __ATOMIC_ACQ_REL);          /* even: stable */
}

static int list_units(void) {
    camera_unit_t units[16];
    uint32_t n = 0;
    if (camera_get_supported_cameras(16, &n, units) != EOK) {
        fprintf(stderr, "camera_get_supported_cameras failed\n");
        return 1;
    }
    for (uint32_t i = 0; i < n; i++) {
        camera_handle_t h;
        if (camera_open(units[i], CAMERA_MODE_RO, &h) != EOK) {
            printf("unit %d: (busy or unavailable)\n", (int)units[i]);
            continue;
        }
        camera_frametype_t ft[16];
        uint32_t nft = 0;
        camera_get_supported_vf_frame_types(h, 16, &nft, ft);
        camera_res_t res[32];
        uint32_t nres = 0;
        camera_get_supported_vf_resolutions(h, 32, &nres, res);
        printf("unit %d: frametypes", (int)units[i]);
        for (uint32_t k = 0; k < nft; k++) printf(" %d", (int)ft[k]);
        printf("  (NV12=%d YCBYCR=%d)\n  resolutions:", (int)CAMERA_FRAMETYPE_NV12,
               (int)CAMERA_FRAMETYPE_YCBYCR);
        for (uint32_t k = 0; k < nres; k++) printf(" %ux%u", res[k].width, res[k].height);
        printf("\n");
        camera_close(h);
    }
    return 0;
}

int main(int argc, char **argv) {
    int unit = 5, want_w = 640, want_h = 480, opt;
    const char *name = "camgrab_fwd";
    while ((opt = getopt(argc, argv, "u:n:w:h:l")) != -1) {
        switch (opt) {
            case 'u': unit = atoi(optarg); break;
            case 'n': name = optarg; break;
            case 'w': want_w = atoi(optarg); break;
            case 'h': want_h = atoi(optarg); break;
            case 'l': return list_units();
            default:
                fprintf(stderr, "usage: %s [-u unit] [-n shm_name] [-w 640 -h 480] | -l\n",
                        argv[0]);
                return 2;
        }
    }

    camera_handle_t h = CAMERA_HANDLE_INVALID;
    int err = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &h);
    if (err != EOK) { fprintf(stderr, "camera_open(%d): %s\n", unit, strerror(err)); return 1; }

    /* NV12 is half the bytes of YUYV; take YUYV if that's all there is */
    camera_frametype_t ft[16];
    uint32_t nft = 0;
    camera_get_supported_vf_frame_types(h, 16, &nft, ft);
    camera_frametype_t chosen = CAMERA_FRAMETYPE_UNSPECIFIED;
    for (uint32_t i = 0; i < nft; i++) {
        if (ft[i] == CAMERA_FRAMETYPE_NV12) { chosen = ft[i]; break; }
        if (ft[i] == CAMERA_FRAMETYPE_YCBYCR) chosen = ft[i];
    }
    if (chosen == CAMERA_FRAMETYPE_UNSPECIFIED) {
        fprintf(stderr, "camera offers neither NV12 nor YCbYCr (%u types); try -l\n", nft);
        return 1;
    }
    g_fmt = chosen == CAMERA_FRAMETYPE_NV12 ? FMT_NV12 : FMT_YUYV;
    camera_set_vf_property(h, CAMERA_IMGPROP_FORMAT, chosen);
    camera_set_vf_property(h, CAMERA_IMGPROP_CREATEWINDOW, 0);
    if (camera_set_vf_property(h, CAMERA_IMGPROP_WIDTH, want_w, CAMERA_IMGPROP_HEIGHT,
                               want_h) != EOK)
        fprintf(stderr, "warn: %dx%d not accepted, keeping the camera default\n", want_w, want_h);
    int w = 0, hgt = 0;
    camera_get_vf_property(h, CAMERA_IMGPROP_WIDTH, &w, CAMERA_IMGPROP_HEIGHT, &hgt);

    uint32_t frame_bytes = g_fmt == FMT_NV12 ? (uint32_t)(w * hgt * 3 / 2) : (uint32_t)(w * hgt * 2);
    char shm_name[128];
    snprintf(shm_name, sizeof shm_name, "/%s", name);
    shm_unlink(shm_name);
    int fd = shm_open(shm_name, O_RDWR | O_CREAT, 0666);
    if (fd < 0) { perror("shm_open"); return 1; }
    size_t total = HDR + frame_bytes;
    if (ftruncate(fd, (off_t)total) != 0) { perror("ftruncate"); return 1; }
    g_map = mmap(NULL, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (g_map == MAP_FAILED) { perror("mmap"); return 1; }
    memset(g_map, 0, HDR);
    hdr_t *hd = (hdr_t *)g_map;
    hd->version = 1; hd->width = (uint32_t)w; hd->height = (uint32_t)hgt;
    hd->format = g_fmt; hd->frame_bytes = frame_bytes; hd->pid = (uint32_t)getpid();
    __atomic_store_n(&hd->magic, MAGIC, __ATOMIC_RELEASE);      /* readers wait for this */

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    err = camera_start_viewfinder(h, on_frame, NULL, NULL);
    if (err != EOK) { fprintf(stderr, "camera_start_viewfinder: %s\n", strerror(err)); return 1; }
    printf("camgrab: unit %d -> /dev/shmem/%s  %dx%d %s\n", unit, name, w, hgt,
           g_fmt == FMT_NV12 ? "NV12" : "YUYV");
    fflush(stdout);

    /* Stall watchdog. Right after a previous owner of the camera dies, QNX's camera service
     * still holds it for a few seconds: camera_open and camera_start_viewfinder succeed, but no
     * frame ever arrives (seen on the Pi: 0 fps forever after a quick restart). Exit instead of
     * hanging, so the supervisor restarts us with backoff once the camera is free. */
    uint64_t last_seq = 0;
    int idle_s = 0, rc = 0;
    while (!g_stop) {
        sleep(1);
        uint64_t s = __atomic_load_n(&hd->seq, __ATOMIC_ACQUIRE);
        idle_s = (s == last_seq) ? idle_s + 1 : 0;
        last_seq = s;
        if (idle_s >= (s == 0 ? 5 : 3)) {
            fprintf(stderr, "camgrab: no frames from unit %d for %d s; exiting so the "
                    "supervisor restarts us\n", unit, idle_s);
            rc = 2;
            break;
        }
    }

    camera_stop_viewfinder(h);
    camera_close(h);
    munmap(g_map, total);
    shm_unlink(shm_name);
    return rc;
}
