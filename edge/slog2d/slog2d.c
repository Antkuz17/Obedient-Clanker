/* slog2d: the drone's processes -> QNX's system log (slog2).
 *
 * Python can't call slog2 without guessing struct layouts, so the edge core and the supervisor
 * send short UDP lines here (edge/slog.py) and this relay writes them to the "hope" buffer set:
 *
 *   "<L> <text>"   L = E (error), W (warning), N (notice), I (info); anything else = info
 *
 * Watch it live on the Pi:  slog2info -w -b hope
 * Build (QNX):  cc -O2 -o edge/slog2d/slog2d edge/slog2d/slog2d.c /usr/lib/libslog2.so.1 -lsocket
 * Run:          ./edge/slog2d/slog2d [port, default 7020]   (listens on 127.0.0.1 only)
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/slog2.h>
#include <sys/socket.h>
#include <unistd.h>

int main(int argc, char **argv) {
    int port = argc > 1 ? atoi(argv[1]) : 7020;

    slog2_buffer_set_config_t cfg;
    slog2_buffer_t buf[1];
    memset(&cfg, 0, sizeof cfg);
    cfg.num_buffers = 1;
    cfg.buffer_set_name = "hope";
    cfg.verbosity_level = SLOG2_INFO;
    cfg.buffer_config[0].buffer_name = "drone";
    cfg.buffer_config[0].num_pages = 8;
    if (slog2_register(&cfg, buf, 0) == -1) {
        perror("slog2d: slog2_register");
        return 1;
    }

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (s < 0 || bind(s, (struct sockaddr *)&a, sizeof a) < 0) {
        perror("slog2d: bind");
        return 1;
    }
    slog2c(buf[0], 0, SLOG2_INFO, "slog2d: drone log started");
    printf("slog2d: 127.0.0.1:%d -> slog2 buffer set 'hope'\n", port);
    fflush(stdout);

    char m[1024];
    for (;;) {
        ssize_t n = recv(s, m, sizeof m - 1, 0);
        if (n <= 0)
            continue;
        m[n] = 0;
        while (n > 0 && (m[n - 1] == '\n' || m[n - 1] == '\r'))
            m[--n] = 0;
        unsigned char sev = SLOG2_INFO;
        const char *text = m;
        if (n > 2 && m[1] == ' ') {
            switch (m[0]) {
            case 'E': sev = SLOG2_ERROR; break;
            case 'W': sev = SLOG2_WARNING; break;
            case 'N': sev = SLOG2_NOTICE; break;
            default: break;
            }
            text = m + 2;
        }
        slog2c(buf[0], 0, sev, text);
    }
}
