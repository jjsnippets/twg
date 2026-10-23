#ifndef DEMO_LINK_H
#define DEMO_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include "demo_imu.h"

#define DEMO_DEFAULT_PORT 8765u

typedef struct {
    pthread_t thread;
    pthread_mutex_t mutex;

    DemoImuSnapshot latest;
    uint64_t session_id;

    uint32_t pending_tare;
    int listener;

    bool running;
    bool exit_requested;
    bool tare_pending;
    bool started;
} DemoLink;

bool demo_link_start(DemoLink *link, uint16_t port,
                     const DemoImuSnapshot *initial);

void demo_link_publish(DemoLink *link, const DemoImuSnapshot *snapshot);
bool demo_link_take_tare(DemoLink *link, uint32_t *request_id);
bool demo_link_exit_requested(DemoLink *link);

/* Call after sensor cleanup; the worker then attempts to send BYE. */
void demo_link_stop(DemoLink *link);

#endif /* DEMO_LINK_H */
