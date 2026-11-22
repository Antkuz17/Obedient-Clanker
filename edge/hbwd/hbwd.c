/*
 * hbwd: the real-time heartbeat watchdog (review D: "don't claim hard real time from Python
 * asyncio; write the small watchdog/heartbeat loop in C at SCHED_FIFO, pinned to a core").
 *
 * The Python edge core "pets" this process every safety tick (50 Hz) over localhost UDP:
 *     "PET <seq> <armed 0|1> <ours 0|1>\n"      ours = FC is in GUIDED or LAND (our modes)
 * hbwd runs its own 50 Hz loop at top SCHED_FIFO and:
 *   - while pets are fresh: sends the GCS HEARTBEAT to the flight controller (2 Hz), so the FC's
 *     own GCS failsafe (FS_GCS_ENABLE=5, FS_GCS_TIMEOUT=2) stays quiet;
 *   - when pets stop for -t ms (core hung, killed, or starved): stops the heartbeat and, if the
 *     last pet said armed + ours, commands LAND right away (3 sends, 100 ms apart) instead of
 *     waiting ~2 s for the FC failsafe. If the pilot has another mode it does NOT touch the mode
 *     (the pilot always wins, review2 #1); the FC failsafe is still the backstop;
 *   - prints one JSON line per event and a stats line every -s seconds with its own wake-up
 *     lateness (p-buckets, max): the jitter graph of the actual safety loop. The loop never does
 *     I/O itself: lines go through a lock-free ring to a normal-priority logger thread (a busy
 *     SD card once stalled the loop 433 ms when it printed directly).
 *
 * MAVLink goes to the USB relay's aux port (tools/usb_relay.py --aux 127.0.0.1:14556), which
 * writes it to the FC. MAVLink v1 frames, sysid 255 (the GCS id ArduPilot's failsafe watches).
 *
 * Build:  cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lsocket   (QNX on the Pi: sockets live in libsocket)
 *         cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lpthread  (Linux, macOS)
 * Run:    ./edge/hbwd/hbwd -p 60 -c 3                       (as root for FIFO + pinning)
 *         python3 -m edge --mav udpin:127.0.0.1:14555 --hbwd 127.0.0.1:14560 ...
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#ifdef __QNX__
#include <sys/neutrino.h>
#endif

/* --- MAVLink v1 encoding (just the two messages we need) ------------------------------- */
#define MSG_HEARTBEAT 0
#define CRC_HEARTBEAT 50
#define MSG_SET_MODE 11
#define CRC_SET_MODE 89
#define COPTER_LAND 9
#define MAV_TYPE_GCS 6
#define MAV_AUTOPILOT_INVALID 8
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1

static uint8_t g_seq;
static uint8_t g_sysid = 255, g_compid = 190;

static void crc_acc(uint8_t b, uint16_t *crc) {
    uint8_t t = b ^ (uint8_t)(*crc & 0xff);
    t ^= (uint8_t)(t << 4);
    *crc = (uint16_t)((*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4));
}

/* returns frame length */
static size_t mav_frame(uint8_t *out, uint8_t msgid, uint8_t crc_extra, const uint8_t *payload,
                        uint8_t len) {
    out[0] = 0xFE;
    out[1] = len;
    out[2] = g_seq++;
    out[3] = g_sysid;
    out[4] = g_compid;
    out[5] = msgid;
    memcpy(out + 6, payload, len);
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 6u + len; i++) crc_acc(out[i], &crc);
    crc_acc(crc_extra, &crc);
    out[6 + len] = (uint8_t)(crc & 0xff);
    out[7 + len] = (uint8_t)(crc >> 8);
    return 8u + len;
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff;
}

static size_t mav_heartbeat(uint8_t *out) {
    uint8_t p[9] = {0};
    put_u32(p, 0);              /* custom_mode */
    p[4] = MAV_TYPE_GCS;        /* type */
    p[5] = MAV_AUTOPILOT_INVALID;
    p[6] = 0;                   /* base_mode */
    p[7] = 0;                   /* system_status */
    p[8] = 3;                   /* mavlink_version */
    return mav_frame(out, MSG_HEARTBEAT, CRC_HEARTBEAT, p, sizeof p);
}

static size_t mav_set_mode_land(uint8_t *out, uint8_t target) {
    uint8_t p[6];
    put_u32(p, COPTER_LAND);    /* custom_mode */
    p[4] = target;              /* target_system */
    p[5] = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    return mav_frame(out, MSG_SET_MODE, CRC_SET_MODE, p, sizeof p);
}

/* --- time ------------------------------------------------------------------------------ */
static int64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void sleep_until(int64_t t_ns) {
#ifdef __APPLE__ /* no clock_nanosleep on macOS: relative sleep (dev only) */
    int64_t d = t_ns - now_ns();
    if (d > 0) {
        struct timespec ts = {d / 1000000000LL, d % 1000000000LL};
        nanosleep(&ts, NULL);
    }
#else
    struct timespec ts = {t_ns / 1000000000LL, t_ns % 1000000000LL};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {}
#endif
}

static double wall_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* --- real time ------------------------------------------------------------------------- */
static int set_fifo(int prio) {
    if (prio <= 0) return 0;
    struct sched_param sp = {.sched_priority = prio};
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0; /* the loop thread only */
}

static int pin_cpu(int cpu) {
    if (cpu < 0) return 0;
#if defined(__QNX__)
    unsigned mask = 1u << cpu;
    return ThreadCtl(_NTO_TCTL_RUNMASK, (void *)(uintptr_t)mask) != -1;
#elif defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return sched_setaffinity(0, sizeof set, &set) == 0;
#else
    (void)cpu;
    return 0;
#endif
}

/* --- lateness histogram (same buckets as tools/jitter.py) ------------------------------ */
static const double EDGES_MS[] = {0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50};
#define NB (sizeof EDGES_MS / sizeof EDGES_MS[0] + 1)
typedef struct {
    uint64_t n, over_2ms, counts[NB];
    double max_ms;
} hist_t;

static void hist_add(hist_t *h, double ms) {
    size_t i = 0;
    while (i < NB - 1 && ms >= EDGES_MS[i]) i++;
    h->counts[i]++;
    h->n++;
    if (ms > 2.0) h->over_2ms++;
    if (ms > h->max_ms) h->max_ms = ms;
}

/* --- logging without blocking the loop -------------------------------------------------- */
/* The loop never does I/O. stdout is a log file on the SD card, and a write there can block
 * for hundreds of ms while the card is busy (measured on the Pi, 2026-11-22: 433 ms loop
 * stalls while other processes hit the card). So the loop formats each line into this ring
 * (single producer: the loop; single consumer: the logger thread) and a normal-priority
 * thread writes them. A full ring drops the line and counts it; the loop never waits. */
#define LOGQ 64
#define LOGLEN 512
static char g_logq[LOGQ][LOGLEN];
static unsigned g_head, g_tail;          /* accessed with __atomic builtins */
static unsigned long g_log_dropped;
static volatile sig_atomic_t g_log_done;

static void logq(const char *fmt, ...) {
    unsigned h = __atomic_load_n(&g_head, __ATOMIC_RELAXED);
    if (h - __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE) >= LOGQ) {
        g_log_dropped++;
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_logq[h % LOGQ], LOGLEN, fmt, ap);
    va_end(ap);
    __atomic_store_n(&g_head, h + 1, __ATOMIC_RELEASE);
}

static void *logger(void *arg) {
    (void)arg;
    for (;;) {
        unsigned t = __atomic_load_n(&g_tail, __ATOMIC_RELAXED);
        unsigned h = __atomic_load_n(&g_head, __ATOMIC_ACQUIRE);
        while (t != h) {
            fputs(g_logq[t % LOGQ], stdout);
            t++;
            __atomic_store_n(&g_tail, t, __ATOMIC_RELEASE);
        }
        fflush(stdout);
        if (g_log_done && t == __atomic_load_n(&g_head, __ATOMIC_ACQUIRE)) return NULL;
        usleep(20000);
    }
}

static void hist_print(const char *kind, const hist_t *h, const char *extra) {
    char counts[NB * 21 + 1];
    size_t off = 0;
    for (size_t i = 0; i < NB && off < sizeof counts; i++)
        off += (size_t)snprintf(counts + off, sizeof counts - off, "%s%llu", i ? "," : "",
                                (unsigned long long)h->counts[i]);
    logq("{\"t\": %.3f, \"kind\": \"%s\", \"n\": %llu, \"max_ms\": %.3f, \"over_2ms\": %llu, "
         "\"edges_ms\": [0.1,0.25,0.5,1,2,5,10,20,50], \"counts\": [%s]%s, \"log_dropped\": %lu}\n",
         wall_s(), kind, (unsigned long long)h->n, h->max_ms, (unsigned long long)h->over_2ms,
         counts, extra, g_log_dropped);
}

/* --- main ------------------------------------------------------------------------------ */
static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

static int parse_addr(const char *s, struct sockaddr_in *a) {
    char host[64];
    const char *c = strrchr(s, ':');
    if (!c || (size_t)(c - s) >= sizeof host) return 0;
    memcpy(host, s, (size_t)(c - s));
    host[c - s] = 0;
    memset(a, 0, sizeof *a);
    a->sin_family = AF_INET;
    a->sin_port = htons((uint16_t)atoi(c + 1));
    return inet_pton(AF_INET, host, &a->sin_addr) == 1;
}

static void usage(void) {
    fprintf(stderr,
            "hbwd [-l pet_addr 127.0.0.1:14560] [-f fc_addr 127.0.0.1:14556] [-p fifo_prio]\n"
            "     [-c cpu] [-t pet_timeout_ms 250] [-r hz 50] [-s stats_s 1] [-T target_sys 1]\n");
}

int main(int argc, char **argv) {
    const char *pet_s = "127.0.0.1:14560", *fc_s = "127.0.0.1:14556";
    int prio = 0, cpu = -1, timeout_ms = 250, hz = 50, target = 1;
    double stats_s = 1.0;
    int opt;
    while ((opt = getopt(argc, argv, "l:f:p:c:t:r:s:T:h")) != -1) {
        switch (opt) {
        case 'l': pet_s = optarg; break;
        case 'f': fc_s = optarg; break;
        case 'p': prio = atoi(optarg); break;
        case 'c': cpu = atoi(optarg); break;
        case 't': timeout_ms = atoi(optarg); break;
        case 'r': hz = atoi(optarg); break;
        case 's': stats_s = atof(optarg); break;
        case 'T': target = atoi(optarg); break;
        default: usage(); return 2;
        }
    }
    struct sockaddr_in pet_a, fc_a;
    if (!parse_addr(pet_s, &pet_a) || !parse_addr(fc_s, &fc_a) || hz <= 0) {
        usage();
        return 2;
    }
    int ps = socket(AF_INET, SOCK_DGRAM, 0), fs = socket(AF_INET, SOCK_DGRAM, 0);
    if (ps < 0 || fs < 0 || bind(ps, (struct sockaddr *)&pet_a, sizeof pet_a) != 0) {
        fprintf(stderr, "hbwd: bind %s: %s\n", pet_s, strerror(errno));
        return 1;
    }
    fcntl(ps, F_SETFL, fcntl(ps, F_GETFL) | O_NONBLOCK);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    /* the logger starts before this thread goes real-time and gets pinned, so it inherits a
     * normal priority and no CPU pin: its writes can block without touching the loop */
    pthread_t log_th;
    int log_ok = pthread_create(&log_th, NULL, logger, NULL) == 0;
    int fifo_ok = set_fifo(prio), pin_ok = pin_cpu(cpu);
    logq("{\"t\": %.3f, \"kind\": \"start\", \"pid\": %d, \"hz\": %d, \"prio\": %d, "
         "\"prio_applied\": %s, \"cpu\": %d, \"pinned\": %s, \"pet_timeout_ms\": %d, "
         "\"logger\": %s}\n",
         wall_s(), (int)getpid(), hz, prio, fifo_ok ? "true" : "false", cpu,
         pin_ok ? "true" : "false", timeout_ms, log_ok ? "true" : "false");

    const int64_t period = 1000000000LL / hz, timeout = (int64_t)timeout_ms * 1000000LL;
    const int64_t hb_period = 500000000LL, land_gap = 100000000LL;
    int64_t next = now_ns() + period, last_pet = 0, last_hb = 0, last_stats = now_ns();
    int64_t lost_at = 0, last_land = 0;
    int have_pet = 0, lost = 0, armed = 0, ours = 0, land_left = 0;
    unsigned long pets = 0, hbs = 0, lands = 0;
    hist_t total = {0}, win = {0};
    uint8_t buf[64], frame[32];

    while (!g_stop) {
        sleep_until(next);
        int64_t t = now_ns();
        double late_ms = (t - next) / 1e6;
        hist_add(&total, late_ms);
        hist_add(&win, late_ms);
        next += period;
        if (t > next + period) next = t + period; /* badly overrun: resync */

        /* drain pets */
        for (;;) {
            ssize_t n = recv(ps, buf, sizeof buf - 1, 0);
            if (n <= 0) break;
            buf[n] = 0;
            unsigned long seq;
            int a, o;
            if (sscanf((char *)buf, "PET %lu %d %d", &seq, &a, &o) == 3) {
                armed = a;
                ours = o;
                last_pet = t;
                pets++;
                if (lost) {
                    logq("{\"t\": %.3f, \"kind\": \"core_back\", \"gap_ms\": %.1f}\n", wall_s(),
                         (t - lost_at) / 1e6 + timeout_ms);
                    lost = 0;
                    land_left = 0;
                }
                have_pet = 1;
            }
        }

        if (have_pet && !lost && t - last_pet > timeout) {
            lost = 1;
            lost_at = t;
            land_left = (armed && ours) ? 3 : 0;
            logq("{\"t\": %.3f, \"kind\": \"core_lost\", \"gap_ms\": %.1f, \"armed\": %s, "
                 "\"ours\": %s, \"action\": \"%s\"}\n",
                 wall_s(), (t - last_pet) / 1e6, armed ? "true" : "false",
                 ours ? "true" : "false",
                 land_left ? "land" : (armed ? "none (pilot mode): FC failsafe" : "none (disarmed)"));
        }
        if (have_pet && !lost && t - last_hb >= hb_period) {
            size_t n = mav_heartbeat(frame);
            sendto(fs, frame, n, 0, (struct sockaddr *)&fc_a, sizeof fc_a);
            last_hb = t;
            hbs++;
        }
        if (land_left > 0 && t - last_land >= land_gap) {
            size_t n = mav_set_mode_land(frame, (uint8_t)target);
            sendto(fs, frame, n, 0, (struct sockaddr *)&fc_a, sizeof fc_a);
            last_land = t;
            lands++;
            land_left--;
            logq("{\"t\": %.3f, \"kind\": \"land_sent\", \"after_lost_ms\": %.1f}\n", wall_s(),
                 (t - lost_at) / 1e6);
        }
        if ((t - last_stats) / 1e9 >= stats_s) {
            char extra[160];
            snprintf(extra, sizeof extra,
                     ", \"pets\": %lu, \"heartbeats\": %lu, \"lands\": %lu, \"core\": \"%s\"",
                     pets, hbs, lands, !have_pet ? "waiting" : (lost ? "lost" : "ok"));
            hist_print("stats", &win, extra);
            memset(&win, 0, sizeof win);
            last_stats = t;
        }
    }
    hist_print("final", &total, "");
    g_log_done = 1;
    if (log_ok) pthread_join(log_th, NULL);
    return 0;
}
