#define _POSIX_C_SOURCE 200809L

#include "demo_link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SEND_PERIOD_NS  UINT64_C(20000000)
#define SEND_TIMEOUT_NS UINT64_C(200000000)
#define FRESH_NS        UINT64_C(250000000)

static bool running(DemoLink *link)
{
    bool value;

    pthread_mutex_lock(&link->mutex);
    value = link->running;
    pthread_mutex_unlock(&link->mutex);
    return value;
}

static void request_exit(DemoLink *link)
{
    pthread_mutex_lock(&link->mutex);
    link->exit_requested = true;
    pthread_mutex_unlock(&link->mutex);
}

static bool nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);

    return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

static bool send_bounded(int fd, const char *data, size_t length)
{
    size_t offset = 0;
    uint64_t deadline = demo_now_ns() + SEND_TIMEOUT_NS;

    while (offset < length) {
        ssize_t count = send(fd, data + offset, length - offset,
                             MSG_NOSIGNAL);

        if (count > 0) {
            offset += (size_t)count;
            continue;
        }

        if (count == 0) {
            return false;
        }

        if (errno == EINTR) {
            continue;
        }

        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            return false;
        }

        if (demo_now_ns() >= deadline) {
            return false;
        }

        {
            struct pollfd pfd = { fd, POLLOUT, 0 };
            int rc = poll(&pfd, 1, 10);

            if (rc < 0 && errno != EINTR) {
                return false;
            }

            if (rc > 0 &&
                (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                return false;
            }
        }
    }

    return true;
}

static bool send_snapshot(DemoLink *link, int fd)
{
    DemoImuSnapshot snapshot;
    char line[384];
    uint64_t now;
    uint64_t age_ms;
    bool valid;
    int length;

    pthread_mutex_lock(&link->mutex);
    snapshot = link->latest;
    pthread_mutex_unlock(&link->mutex);

    now = demo_now_ns();

    if (snapshot.sample_ns != 0 && now >= snapshot.sample_ns) {
        age_ms = (now - snapshot.sample_ns) / UINT64_C(1000000);
        valid = snapshot.valid &&
                now - snapshot.sample_ns <= FRESH_NS;
    } else {
        age_ms = UINT64_C(999999);
        valid = false;
    }

    /*
     * D version serial reference epoch sequence age_ms valid phase
     *   tare_id tare_result error accuracy qw qx qy qz
     */
    length = snprintf(
        line, sizeof(line),
        "D 1 %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu64
        " %" PRIu64 " %u %u %" PRIu32 " %d %u %u"
        " %.9g %.9g %.9g %.9g\n",
        snapshot.serial,
        snapshot.reference,
        snapshot.epoch,
        snapshot.sequence,
        age_ms,
        valid ? 1u : 0u,
        (unsigned)snapshot.phase,
        snapshot.tare_id,
        snapshot.tare_result,
        (unsigned)snapshot.error,
        snapshot.accuracy,
        (double)snapshot.q[0],
        (double)snapshot.q[1],
        (double)snapshot.q[2],
        (double)snapshot.q[3]);

    if (length < 0 || (size_t)length >= sizeof(line)) {
        return false;
    }

    return send_bounded(fd, line, (size_t)length);
}

static bool command(DemoLink *link, int fd, const char *line,
                    uint32_t *last_tare)
{
    char *end;
    unsigned long value;
    uint32_t id;
    bool accepted;

    if (strcmp(line, "X") == 0) {
        request_exit(link);
        return true;
    }

    if (line[0] != 'T' || line[1] != ' ' ||
        line[2] < '0' || line[2] > '9') {
        return false;
    }

    errno = 0;
    value = strtoul(line + 2, &end, 10);

    if (errno != 0 || *end != '\0' ||
        value == 0 || value > UINT32_MAX) {
        return false;
    }

    id = (uint32_t)value;

    if (id == *last_tare) {
        return true;
    }

    *last_tare = id;

    pthread_mutex_lock(&link->mutex);

    accepted = !link->tare_pending &&
               !link->exit_requested &&
               link->latest.phase == DEMO_PHASE_TRACKING;

    if (accepted) {
        link->pending_tare = id;
        link->tare_pending = true;
    }

    pthread_mutex_unlock(&link->mutex);

    if (!accepted) {
        char reply[48];
        int length = snprintf(reply, sizeof(reply),
                              "E %" PRIu32 "\n", id);

        return length > 0 &&
               (size_t)length < sizeof(reply) &&
               send_bounded(fd, reply, (size_t)length);
    }

    return true;
}

static void drop_client(DemoLink *link, int *client)
{
    if (*client >= 0) {
        close(*client);
        *client = -1;
    }

    pthread_mutex_lock(&link->mutex);
    link->tare_pending = false;
    pthread_mutex_unlock(&link->mutex);
}

static void *worker(void *argument)
{
    DemoLink *link = argument;
    int client = -1;
    char input[128];
    size_t used = 0;
    uint32_t last_tare = 0;
    uint64_t next_send = 0;

    while (running(link)) {
        struct pollfd pfd;
        int rc;

        pfd.fd = client >= 0 ? client : link->listener;
        pfd.events = POLLIN;
        pfd.revents = 0;

        rc = poll(&pfd, 1, 5);

        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }

            request_exit(link);
            break;
        }

        if (client < 0) {
            if (rc > 0 &&
                (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                request_exit(link);
                break;
            }

            if (rc > 0 && (pfd.revents & POLLIN) != 0) {
                int one = 1;
                int buffer_size = 4096;
                char hello[80];
                int length;

                client = accept(link->listener, NULL, NULL);

                if (client < 0) {
                    continue;
                }

                if (!nonblocking(client)) {
                    drop_client(link, &client);
                    continue;
                }

                (void)setsockopt(client, IPPROTO_TCP, TCP_NODELAY,
                                 &one, sizeof(one));
                (void)setsockopt(client, SOL_SOCKET, SO_KEEPALIVE,
                                 &one, sizeof(one));
                (void)setsockopt(client, SOL_SOCKET, SO_SNDBUF,
                                 &buffer_size, sizeof(buffer_size));

#ifdef TCP_USER_TIMEOUT
                {
                    unsigned timeout_ms = 1500u;
                    (void)setsockopt(client, IPPROTO_TCP, TCP_USER_TIMEOUT,
                                     &timeout_ms, sizeof(timeout_ms));
                }
#endif

                used = 0;
                last_tare = 0;
                next_send = 0;

                length = snprintf(hello, sizeof(hello),
                                  "H 1 %016" PRIx64 "\n",
                                  link->session_id);

                if (length < 0 || (size_t)length >= sizeof(hello) ||
                    !send_bounded(client, hello, (size_t)length)) {
                    drop_client(link, &client);
                }
            }

            continue;
        }

        if (rc > 0 && (pfd.revents & POLLIN) != 0) {
            char buffer[512];
            ssize_t count = recv(client, buffer, sizeof(buffer), 0);

            if (count <= 0) {
                if (count < 0 &&
                    (errno == EINTR || errno == EAGAIN ||
                     errno == EWOULDBLOCK)) {
                    continue;
                }

                drop_client(link, &client);
                used = 0;
                continue;
            }

            for (ssize_t i = 0; i < count; ++i) {
                unsigned char ch = (unsigned char)buffer[i];

                if (ch == '\r') {
                    continue;
                }

                if (ch == '\n') {
                    input[used] = '\0';

                    if (!command(link, client, input, &last_tare)) {
                        drop_client(link, &client);
                        break;
                    }

                    used = 0;
                } else {
                    if (ch < 32u || ch > 126u ||
                        used + 1u >= sizeof(input)) {
                        drop_client(link, &client);
                        break;
                    }

                    input[used++] = (char)ch;
                }
            }
        }

        if (client >= 0 && rc > 0 &&
            (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            drop_client(link, &client);
        }

        if (client >= 0 && demo_now_ns() >= next_send) {
            if (!send_snapshot(link, client)) {
                drop_client(link, &client);
            }

            next_send = demo_now_ns() + SEND_PERIOD_NS;
        }
    }

    /*
     * main calls stop only after closing the IMU. BYE therefore means
     * that normal application-level sensor cleanup has completed.
     */
    if (client >= 0) {
        (void)send_bounded(client, "BYE\n", 4u);
        close(client);
    }

    close(link->listener);
    return NULL;
}

bool demo_link_start(DemoLink *link, uint16_t port,
                     const DemoImuSnapshot *initial)
{
    struct sockaddr_in address;
    int one = 1;
    int rc;
    int saved_errno;

    if (link == NULL || initial == NULL || port == 0) {
        errno = EINVAL;
        return false;
    }

    memset(link, 0, sizeof(*link));
    link->listener = -1;
    link->latest = *initial;
    link->session_id = demo_now_ns() ^
                       ((uint64_t)(unsigned long)getpid() << 32);

    rc = pthread_mutex_init(&link->mutex, NULL);
    if (rc != 0) {
        errno = rc;
        return false;
    }

    link->listener = socket(AF_INET, SOCK_STREAM, 0);
    if (link->listener < 0) {
        goto fail;
    }

    (void)setsockopt(link->listener, SOL_SOCKET, SO_REUSEADDR,
                     &one, sizeof(one));

    if (!nonblocking(link->listener)) {
        goto fail;
    }

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);

    if (bind(link->listener, (struct sockaddr *)&address,
             sizeof(address)) != 0 ||
        listen(link->listener, 1) != 0) {
        goto fail;
    }

    link->running = true;
    rc = pthread_create(&link->thread, NULL, worker, link);

    if (rc != 0) {
        errno = rc;
        goto fail;
    }

    link->started = true;
    return true;

fail:
    saved_errno = errno;

    if (link->listener >= 0) {
        close(link->listener);
    }

    pthread_mutex_destroy(&link->mutex);
    errno = saved_errno;
    return false;
}

void demo_link_publish(DemoLink *link, const DemoImuSnapshot *snapshot)
{
    pthread_mutex_lock(&link->mutex);
    link->latest = *snapshot;
    pthread_mutex_unlock(&link->mutex);
}

bool demo_link_take_tare(DemoLink *link, uint32_t *request_id)
{
    bool pending;

    pthread_mutex_lock(&link->mutex);

    pending = link->tare_pending;
    if (pending) {
        *request_id = link->pending_tare;
        link->tare_pending = false;
    }

    pthread_mutex_unlock(&link->mutex);
    return pending;
}

bool demo_link_exit_requested(DemoLink *link)
{
    bool requested;

    pthread_mutex_lock(&link->mutex);
    requested = link->exit_requested;
    pthread_mutex_unlock(&link->mutex);
    return requested;
}

void demo_link_stop(DemoLink *link)
{
    if (link == NULL || !link->started) {
        return;
    }

    pthread_mutex_lock(&link->mutex);
    link->running = false;
    pthread_mutex_unlock(&link->mutex);

    (void)pthread_join(link->thread, NULL);
    pthread_mutex_destroy(&link->mutex);
    link->started = false;
}
