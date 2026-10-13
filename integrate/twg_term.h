#ifndef TWG_TERM_H
#define TWG_TERM_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>

#define TWG_TERM_QUEUE_CAPACITY 32u

typedef struct {
    char key;
} TwgTermKey;

typedef struct {
    TwgTermKey queue[TWG_TERM_QUEUE_CAPACITY];

    _Atomic uint64_t head;
    _Atomic uint64_t tail;
    _Atomic uint64_t flushFloor;
    _Atomic uint64_t dropped;
    _Atomic uint64_t flushRequested;
    _Atomic uint64_t flushAcknowledged;

    _Atomic bool stopRequested;
    _Atomic bool eofSeen;
    _Atomic bool inputError;
    _Atomic bool workerSchedOther;

    pthread_t worker;
    int fd;

    bool initialized;
    bool workerStarted;
    bool rawMode;
    bool termiosSaved;

    struct termios savedTermios;
} TwgTerm;

bool twg_term_init(TwgTerm *term, int fd);
bool twg_term_start(TwgTerm *term);

void twg_term_request_stop(TwgTerm *term);
bool twg_term_join(TwgTerm *term);
void twg_term_destroy(TwgTerm *term);

bool twg_term_dequeue(TwgTerm *term, TwgTermKey *key);

uint64_t twg_term_request_flush(TwgTerm *term);
bool twg_term_flush_complete(const TwgTerm *term, uint64_t ticket);

bool twg_term_eof_seen(const TwgTerm *term);
bool twg_term_input_error(const TwgTerm *term);
bool twg_term_worker_sched_other(const TwgTerm *term);
uint64_t twg_term_dropped(const TwgTerm *term);

#endif /* TWG_TERM_H */
