#define _POSIX_C_SOURCE 200809L
#include <fcntl.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app/imu_logger.h"

static int s_failures;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++s_failures;                                                    \
        }                                                                    \
    } while (0)

static ImuLogger_t g_log;

typedef struct {
    _Atomic int writes;
    _Atomic int syncs;
    _Atomic int gateOpen;
    _Atomic int blocked;
    int failAt;
    int syncFails;
    uint64_t lines;
    uint64_t lastSeq;
    uint64_t maxDrops;
    uint64_t lastDrops;
    int orderOk;
    int headerOk;
} TestSink_t;

static void ms(long n)
{
    struct timespec ts = { 0, n * 1000000L };
    nanosleep(&ts, NULL);
}

static uint64_t field_u64(const char *line, unsigned idx)
{
    unsigned i = 0u;

    while (i < idx) {
        line = strchr(line, ',');
        if (line == NULL) return 0u;
        ++line;
        ++i;
    }
    return strtoull(line, NULL, 10);
}

static int sink_write(void *ctx, const char *data, size_t length)
{
    TestSink_t *s = (TestSink_t *)ctx;
    int n = atomic_fetch_add(&s->writes, 1) + 1;
    const char *p = data;
    const char *end = data + length;

    if (n == s->failAt) {
        return -1;
    }
    if (n > 1 && atomic_load(&s->gateOpen) == 0) {
        atomic_store(&s->blocked, 1);
        while (atomic_load(&s->gateOpen) == 0) {
            ms(1);
        }
    }
    if (length == 0u || data[length - 1u] != '\n') {
        s->orderOk = 0;
    }
    while (p < end) {
        const char *nl = memchr(p, '\n', (size_t)(end - p));
        char line[2048];
        size_t len;

        if (nl == NULL || (size_t)(nl - p) >= sizeof(line)) {
            s->orderOk = 0;
            break;
        }
        len = (size_t)(nl - p);
        memcpy(line, p, len);
        line[len] = '\0';
        if (n == 1 && p == data) {
            s->headerOk = strncmp(line, "schema_ver,pub_seq,", 19) == 0;
        } else {
            uint64_t seq = field_u64(line, 1);
            uint64_t drops = field_u64(line, 16);

            if (seq <= s->lastSeq || drops < s->lastDrops) s->orderOk = 0;
            s->lastSeq = seq;
            s->lastDrops = drops;
            if (drops > s->maxDrops) s->maxDrops = drops;
            ++s->lines;
        }
        p = nl + 1;
    }
    return 0;
}

static int sink_sync(void *ctx)
{
    TestSink_t *s = (TestSink_t *)ctx;

    atomic_fetch_add(&s->syncs, 1);
    return s->syncFails ? -1 : 0;
}

static void sink_init(TestSink_t *s, int gateOpen)
{
    memset(s, 0, sizeof(*s));
    s->orderOk = 1;
    atomic_store(&s->gateOpen, gateOpen);
}

static ImuLoggerSink_t make_sink(TestSink_t *s)
{
    ImuLoggerSink_t k;

    k.write = sink_write;
    k.sync = sink_sync;
    k.ctx = s;
    return k;
}

static ImuPublicationRecord_t rec(uint64_t seq)
{
    ImuPublicationRecord_t r;

    memset(&r, 0, sizeof(r));
    r.version = IMU_PUBLICATION_CONTRACT_VERSION;
    r.publicationSeq = seq;
    r.configurationEpoch = 3u;
    r.readerState = IMU_READER_STATE_OPERATIONAL;
    return r;
}

static int wait_blocked(TestSink_t *s)
{
    int i;

    for (i = 0; i < 2000; ++i) {
        if (atomic_load(&s->blocked) != 0) return 1;
        ms(1);
    }
    return 0;
}

static void test_basic(void)
{
    TestSink_t s;
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r = rec(1);
    unsigned i;

    sink_init(&s, 1);
    k = make_sink(&s);

    CHECK(!imu_logger_init(NULL));
    CHECK(imu_logger_init(&g_log));
    CHECK(imu_logger_stats(&g_log, &st));
    CHECK(st.startResult == IMU_LOGGER_START_NOT_ATTEMPTED);
    CHECK(st.queueCapacity == IMU_LOGGER_QUEUE_CAPACITY);
    CHECK(!imu_logger_enqueue(&g_log, &r));
    CHECK(!imu_logger_drain(&g_log, 1000000ull));
    CHECK(!imu_logger_start(&g_log, NULL));

    CHECK(imu_logger_start(&g_log, &k));
    CHECK(!imu_logger_start(&g_log, &k));
    CHECK(s.headerOk);

    r.version = 99u;
    CHECK(!imu_logger_enqueue(&g_log, &r));
    CHECK(!imu_logger_enqueue(&g_log, NULL));

    for (i = 1u; i <= 100u; ++i) {
        r = rec(i);
        CHECK(imu_logger_enqueue(&g_log, &r));
    }
    CHECK(imu_logger_drain(&g_log, 2000000000ull));
    CHECK(imu_logger_drain(&g_log, 0u));
    r = rec(101u);
    CHECK(!imu_logger_enqueue(&g_log, &r));

    CHECK(imu_logger_stats(&g_log, &st));
    CHECK(st.startResult == IMU_LOGGER_START_OK);
    CHECK(st.recordsEnqueued == 100u);
    CHECK(st.recordsWritten == 100u);
    CHECK(st.recordsDropped == 0u);
    CHECK(st.queueHighWater >= 1u && st.queueHighWater <= 100u);
    CHECK(st.writeFailed == 0u && st.shutdownErrors == 0u);
    CHECK(st.drainStarted == 1u && st.drainComplete == 1u && st.drainTimedOut == 0u);
    CHECK(s.lines == 100u && s.lastSeq == 100u && s.orderOk);
    CHECK(atomic_load(&s.syncs) == 1);
    CHECK(!imu_logger_write_failed(&g_log));
}

static void test_overflow_and_drops(void)
{
    TestSink_t s;
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r;
    unsigned i;
    unsigned accepted = 0u;
    int waited;

    sink_init(&s, 0);
    k = make_sink(&s);
    CHECK(imu_logger_init(&g_log));
    CHECK(imu_logger_start(&g_log, &k));

    r = rec(1u);
    CHECK(imu_logger_enqueue(&g_log, &r));
    CHECK(wait_blocked(&s));

    for (i = 2u; i <= 2048u + 1u + 5u; ++i) {
        r = rec(i);
        if (imu_logger_enqueue(&g_log, &r)) ++accepted;
    }
    CHECK(accepted == 2048u);
    CHECK(imu_logger_stats(&g_log, &st));
    CHECK(st.recordsDropped == 5u);
    CHECK(st.queueHighWater == 2048u);

    atomic_store(&s.gateOpen, 1);
    for (waited = 0; waited < 3000; ++waited) {
        imu_logger_stats(&g_log, &st);
        if (st.recordsWritten == 2049u) break;
        ms(1);
    }
    CHECK(st.recordsWritten == 2049u);

    r = rec(5000u);
    CHECK(imu_logger_enqueue(&g_log, &r));
    CHECK(imu_logger_drain(&g_log, 2000000000ull));
    CHECK(imu_logger_stats(&g_log, &st));
    CHECK(st.recordsEnqueued == 2050u);
    CHECK(st.recordsWritten == 2050u);
    CHECK(st.recordsDropped == 5u);
    CHECK(s.maxDrops == 5u && s.lastDrops == 5u && s.lastSeq == 5000u);
    CHECK(s.orderOk);
}

static void test_drain_timeout_retry(void)
{
    TestSink_t s;
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r = rec(1u);

    sink_init(&s, 0);
    k = make_sink(&s);
    CHECK(imu_logger_init(&g_log));
    CHECK(imu_logger_start(&g_log, &k));
    CHECK(imu_logger_enqueue(&g_log, &r));
    CHECK(wait_blocked(&s));

    CHECK(!imu_logger_drain(&g_log, 2000000ull));
    imu_logger_stats(&g_log, &st);
    CHECK(st.drainStarted == 1u && st.drainComplete == 0u && st.drainTimedOut == 1u);

    atomic_store(&s.gateOpen, 1);
    CHECK(imu_logger_drain(&g_log, 2000000000ull));
    imu_logger_stats(&g_log, &st);
    CHECK(st.drainComplete == 1u && st.drainTimedOut == 0u);
    CHECK(st.recordsWritten == 1u);
}

static void test_write_failure(void)
{
    TestSink_t s;
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r;
    unsigned i;

    sink_init(&s, 1);
    s.failAt = 2;
    s.syncFails = 1;
    k = make_sink(&s);
    CHECK(imu_logger_init(&g_log));
    CHECK(imu_logger_start(&g_log, &k));

    for (i = 1u; i <= 5u; ++i) {
        r = rec(i);
        CHECK(imu_logger_enqueue(&g_log, &r));
    }
    for (i = 0u; i < 2000u && !imu_logger_write_failed(&g_log); ++i) ms(1);
    CHECK(imu_logger_write_failed(&g_log));

    for (i = 6u; i <= 10u; ++i) {
        r = rec(i);
        CHECK(imu_logger_enqueue(&g_log, &r));
    }
    CHECK(imu_logger_drain(&g_log, 2000000000ull));
    imu_logger_stats(&g_log, &st);
    CHECK(st.writeFailed == 1u);
    CHECK(st.recordsEnqueued == 10u);
    CHECK(st.recordsWritten == 0u);
    CHECK(st.recordsDropped == 0u);
    CHECK(st.shutdownErrors == 1u);
    CHECK(atomic_load(&s.writes) == 2);
}

static void test_start_failure(void)
{
    TestSink_t s;
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r = rec(1u);

    sink_init(&s, 1);
    s.failAt = 1;
    k = make_sink(&s);
    CHECK(imu_logger_init(&g_log));
    CHECK(!imu_logger_start(&g_log, &k));
    imu_logger_stats(&g_log, &st);
    CHECK(st.startResult == IMU_LOGGER_START_FAILED);
    CHECK(!imu_logger_enqueue(&g_log, &r));
    CHECK(!imu_logger_drain(&g_log, 1000000ull));
    CHECK(!imu_logger_start(&g_log, &k));
}

static void test_fd_sink(void)
{
    char path[] = "/tmp/test_imu_logger_XXXXXX";
    int fd = mkstemp(path);
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r;
    char buf[65536];
    ssize_t n;
    unsigned i, newlines = 0u;
    int bad = -1;

    CHECK(fd >= 0);
    CHECK(!imu_logger_sink_fd(&k, &bad));
    CHECK(imu_logger_sink_fd(&k, &fd));
    CHECK(imu_logger_init(&g_log));
    CHECK(imu_logger_start(&g_log, &k));
    for (i = 1u; i <= 20u; ++i) {
        r = rec(i);
        CHECK(imu_logger_enqueue(&g_log, &r));
    }
    CHECK(imu_logger_drain(&g_log, 2000000000ull));
    imu_logger_stats(&g_log, &st);
    CHECK(st.recordsWritten == 20u && st.shutdownErrors == 0u);

    CHECK(lseek(fd, 0, SEEK_SET) == 0);
    n = read(fd, buf, sizeof(buf));
    CHECK(n > 0);
    for (i = 0u; n > 0 && i < (unsigned)n; ++i) if (buf[i] == '\n') ++newlines;
    CHECK(newlines == 21u);
    CHECK(n > 0 && strncmp(buf, "schema_ver,pub_seq,", 19) == 0);
    close(fd);
    unlink(path);
}

static void test_stress(void)
{
    TestSink_t s;
    ImuLoggerSink_t k;
    ImuLoggerStats_t st;
    ImuPublicationRecord_t r;
    uint64_t i;
    uint64_t ok = 0u, bad = 0u;
    const uint64_t attempts = 100000u;

    sink_init(&s, 1);
    k = make_sink(&s);
    CHECK(imu_logger_init(&g_log));
    CHECK(imu_logger_start(&g_log, &k));

    for (i = 1u; i <= attempts; ++i) {
        r = rec(i);
        if (imu_logger_enqueue(&g_log, &r)) ++ok; else ++bad;
        if ((i % 512u) == 0u) ms(1);
    }
    CHECK(imu_logger_drain(&g_log, 5000000000ull));
    imu_logger_stats(&g_log, &st);
    CHECK(ok + bad == attempts);
    CHECK(st.recordsEnqueued == ok);
    CHECK(st.recordsDropped == bad);
    CHECK(st.recordsWritten == ok);
    CHECK(s.lines == ok);
    CHECK(s.orderOk);
    CHECK(st.queueHighWater <= IMU_LOGGER_QUEUE_CAPACITY);
    printf("info: stress attempts=%llu enqueued=%llu dropped=%llu highwater=%u\n",
           (unsigned long long)attempts, (unsigned long long)ok,
           (unsigned long long)bad, st.queueHighWater);
    printf("info: sizeof(ImuLogger_t)=%zu\n", sizeof(ImuLogger_t));
}

int main(void)
{
    test_basic();
    test_overflow_and_drops();
    test_drain_timeout_retry();
    test_write_failure();
    test_start_failure();
    test_fd_sink();
    test_stress();

    if (s_failures != 0) {
        fprintf(stderr, "test_imu_logger: %d failure(s)\n", s_failures);
        return 1;
    }
    puts("test_imu_logger: PASS");
    return 0;
}
