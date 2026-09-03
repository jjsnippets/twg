#define _POSIX_C_SOURCE 200809L
/*
 * test_report_len.c
 *
 * Verifies the actual report payload lengths the BNO085 delivers for the
 * three sensor types used by the timing tests (Rotation Vector, Linear
 * Acceleration, Gyroscope Calibrated), across every rate in
 * TEST_RATES_HZ[] -- the same sweep style as test_timing_rv/accel/gyro.c.
 *
 * TWO different "lengths" are measured per event, because they answer
 * different questions:
 *
 *   1. sh2_SensorEvent_t.len -- what sh2_lib reports. IMPORTANT: sh2.c
 *      (CEVA v1.4.0) fills this from its STATIC table sh2ReportLens[],
 *      not from the received bytes: ROTATION_VECTOR=14,
 *      LINEAR_ACCELERATION=10, GYROSCOPE_CALIBRATED=10. This column can
 *      therefore only ever print those constants -- but it is still
 *      worth logging because of (2):
 *
 *   2. The SHTP packet length on the wire, decoded here from the 4-byte
 *      SHTP header (bytes 0-1 of the HAL read buffer, little-endian,
 *      bit 15 = continuation flag) by a read() shim below. This is the
 *      number of bytes the planned two-phase CS-controlled read would
 *      actually clock over SPI, and it is measured from the DEVICE, not
 *      from a table.
 *
 * Interpretation guide:
 *   - pkt(ch3) consistently == 4 + table_len  -> static table matches
 *     this firmware; the two-phase read projection holds.
 *   - pkt(ch3) occasionally == 4 + N*table_len -> multiple sensor
 *     reports batched into one SHTP packet (sh2.c walks the payload
 *     with cursor += reportLen; N reports share one packet).
 *   - pkt(ch3) consistently != expected -> firmware report format
 *     differs from the CEVA table; use the WIRE number for the CS-fix
 *     projection, and expect parsing quirks.
 *   - "other" > 0 -> leftover reports from the previous sensor
 *     (disable via reportInterval_us=0 not fully effective) or
 *     unexpected report types arriving on the sensor channel.
 *
 * Per the CEVA sources: sensor input reports arrive on SHTP channel 3
 * (CHAN_SENSORHUB_INPUT in sh2.c); reportId == sh2 sensorId for these
 * reports (sh2_SensorValue.c: value->sensorId = event->reportId).
 *
 * Output:
 *   - report_lens.csv: one row per event, tagged with the configured
 *     sensor label and the SET rate in Hz (so readouts for a single
 *     report type can be concatenated/filtered for later processing),
 *     including event.len, the wire packet length/channel/continuation
 *     flag, the event sequence byte, and the event timestamp.
 *   - stdout: per-run summaries, a final expected-vs-measured table,
 *     and the two-phase read byte projection.
 *
 * Throwaway diagnostic program -- not part of the final application.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>

#include "sh2.h"          /* sh2_open, sh2_setSensorConfig, sh2_SensorEvent_t */
#include "sh2_hal.h"      /* sh2_Hal_t, SH2_HAL_MAX_TRANSFER_IN */
#include "sh2_err.h"      /* SH2_OK */
#include "realtime.h"     /* StartRT, RT_SleepUntil */

extern sh2_Hal_t *sh2_hal_rpi_init(void);

/* ------------------------------------------------------------------ */
/* Tunables (near the top, mirroring test_timing_*.c)                  */
/* ------------------------------------------------------------------ */

/* Sensors to sweep. Add/remove entries as needed; the run loop, CSV and
 * summary table all derive from this array. Expected values are per
 * sh2.c's static sh2ReportLens[] table (+ 4-byte SHTP header). */
typedef struct {
    sh2_SensorId_t id;
    const char    *label;
    unsigned       expectedEvtLen;   /* sh2_SensorEvent_t.len (CEVA table) */
    unsigned       expectedPktLen;   /* wire: 4-byte SHTP header + report   */
} SensorSpec_t;

static const SensorSpec_t TEST_SENSORS[] = {
    { SH2_ROTATION_VECTOR,      "ROTATION_VECTOR",      14, 18 },
    { SH2_LINEAR_ACCELERATION,  "LINEAR_ACCELERATION",  10, 14 },
    { SH2_GYROSCOPE_CALIBRATED, "GYROSCOPE_CALIBRATED", 10, 14 },
};
#define NUM_TEST_SENSORS (sizeof(TEST_SENSORS) / sizeof(TEST_SENSORS[0]))

/* Rates to sweep per sensor. Same list as test_timing_*.c so results are
 * directly comparable; payload length should be rate-INDEPENDENT, so a
 * constant length across all rates is itself a finding. */
static const unsigned TEST_RATES_HZ[] = { 25, 50, 100, 200, 300, 400 };
#define NUM_TEST_RATES (sizeof(TEST_RATES_HZ) / sizeof(TEST_RATES_HZ[0]))

#define TEST_DURATION_SEC     10      /* 2-3 s suffices for length-only checks;
                                       * 10 s matches test_timing_*.c windows */
#define SETTLE_DELAY_SEC      1       /* drain stale reports after reconfig, before logging */
#define RT_PRIORITY           90      /* keep in sync with test_timing_*.c / main.c */
#define LOOP_DT_SEC           0.001   /* 1ms servicing cadence */
#define REPORT_LEN_CSV        "report_lens.csv"

/* Per-byte spidev cost at 3 MHz on this Pi, from the hal_sweep.csv
 * analysis (isolated ioctl timing sweep). Used only for the printed
 * two-phase-read projection, not for any measurement. */
#define MEASURED_US_PER_BYTE  6.2

/* SHTP channel numbers (per sh2.c v1.4.0) */
#define SHTP_CHAN_EXECUTABLE_DEVICE  1
#define SHTP_CHAN_CONTROL            2
#define SHTP_CHAN_SENSOR_INPUT       3
#define SHTP_CHAN_SENSOR_INPUT_WAKE  4

/* ------------------------------------------------------------------ */
/* Distinct-value tracker (payload lengths take few distinct values)   */
/* ------------------------------------------------------------------ */

#define MAX_DISTINCT 8

typedef struct {
    unsigned value[MAX_DISTINCT];
    unsigned count[MAX_DISTINCT];
    unsigned n;
} Distinct_t;

static void noteDistinct(Distinct_t *d, unsigned value)
{
    for (unsigned i = 0; i < d->n; i++) {
        if (d->value[i] == value) {
            d->count[i]++;
            return;
        }
    }
    if (d->n < MAX_DISTINCT) {
        d->value[d->n] = value;
        d->count[d->n] = 1;
        d->n++;
    }
}

static const char *distinctStr(const Distinct_t *d, char *buf, size_t bufLen)
{
    size_t off = 0;
    if ((d == NULL) || (d->n == 0)) {
        snprintf(buf, bufLen, "none");
        return buf;
    }
    for (unsigned i = 0; (i < d->n) && (off < bufLen); i++) {
        int w = snprintf(buf + off, bufLen - off, "%s%ux%u",
                         (i != 0) ? "; " : "", d->value[i], d->count[i]);
        if (w < 0) break;
        off += (size_t)w;
    }
    return buf;
}

static unsigned modeOf(const Distinct_t *d)
{
    unsigned best = 0;
    unsigned bestCount = 0;
    for (unsigned i = 0; i < d->n; i++) {
        if (d->count[i] > bestCount) {
            bestCount = d->count[i];
            best = d->value[i];
        }
    }
    return best;
}

/* ------------------------------------------------------------------ */
/* Run state (declared before the HAL shim, which counts reads)        */
/* ------------------------------------------------------------------ */

static FILE    *sCsv = NULL;
static unsigned sCsvRowIdx = 0;

static sh2_SensorId_t sActiveSensorId = 0xFF;
static const char    *sActiveSensorLabel = "-";
static unsigned       sActiveRateHz = 0;

static unsigned  sRunEvents = 0;
static unsigned  sRunMatch = 0;
static unsigned  sRunOther = 0;
static unsigned  sRunEmptyReads = 0;
static unsigned  sRunDataReads = 0;
static unsigned  sRunChanCounts[8];
static Distinct_t sRunEvtLens;
static Distinct_t sRunPktLens;

/* Per-sensor accumulation across all rates, for the final summary */
static Distinct_t sSensorEvtLens[NUM_TEST_SENSORS];
static Distinct_t sSensorPktLens[NUM_TEST_SENSORS];
static unsigned    sSensorEvents[NUM_TEST_SENSORS];

/* Async event counters */
static unsigned sResetCount = 0;
static unsigned sGetFeatureRespCount = 0;

static void resetRunStats(void)
{
    sRunEvents = 0;
    sRunMatch = 0;
    sRunOther = 0;
    sRunEmptyReads = 0;
    sRunDataReads = 0;
    memset(sRunChanCounts, 0, sizeof(sRunChanCounts));
    memset(&sRunEvtLens, 0, sizeof(sRunEvtLens));
    memset(&sRunPktLens, 0, sizeof(sRunPktLens));
}

/* ------------------------------------------------------------------ */
/* HAL shim: pass read()/write() through to the real HAL, and capture  */
/* the SHTP header of every non-empty read.                            */
/*                                                                     */
/* shtp.c always calls hal->read() with len == SH2_HAL_MAX_TRANSFER_IN */
/* (1024) -- the architectural finding from the earlier investigation. */
/* The first 4 bytes written into the buffer by rpi_read() are the     */
/* SHTP header of the actual packet, which is where the real wire      */
/* length lives.                                                       */
/* ------------------------------------------------------------------ */

static sh2_Hal_t *sRealHal = NULL;
static sh2_Hal_t  sShimHal;

/* Last-read SHTP header (set by shimRead, consumed by sensorCallback) */
static bool     sLastPktValid = false;
static uint16_t sLastPktLen   = 0;
static uint8_t  sLastPktChan  = 0;
static bool     sLastPktCont  = false;

static void captureShtpHeader(const uint8_t *pBuffer, unsigned n)
{
    if (n < 4) {
        sLastPktValid = false;
        return;
    }
    /* SHTP header: bytes 0-1 = packet length (LE, includes this 4-byte
     * header; bit 15 = continuation), byte 2 = channel, byte 3 = seq. */
    uint16_t raw = (uint16_t)(pBuffer[0] | ((uint16_t)pBuffer[1] << 8));
    sLastPktCont  = ((raw & 0x8000u) != 0);
    sLastPktLen   = (uint16_t)(raw & 0x7FFFu);
    sLastPktChan  = pBuffer[2];
    sLastPktValid = true;
}

static int shimOpen(sh2_Hal_t *self)
{
    (void)self;
    return sRealHal->open(sRealHal);
}

static void shimClose(sh2_Hal_t *self)
{
    (void)self;
    sRealHal->close(sRealHal);
}

static int shimWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;
    return sRealHal->write(sRealHal, pBuffer, len);
}

static uint32_t shimGetTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return sRealHal->getTimeUs(sRealHal);
}

static int shimRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;
    int n = sRealHal->read(sRealHal, pBuffer, len, t_us);
    if (n > 0) {
        sRunDataReads++;
        captureShtpHeader(pBuffer, (unsigned)n);
        if (sLastPktValid && (sLastPktChan < 8)) {
            sRunChanCounts[sLastPktChan]++;
        }
    }
    else {
        /* 0 = no data ready; negative = HAL error. Both counted as
         * "empty" here -- this test does not measure timing. */
        sRunEmptyReads++;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* Event ring buffer                                                   */
/*                                                                     */
/* The sensor callback fires inside sh2_service() (same thread as the  */
/* main loop), so no locking is needed; the ring just avoids doing     */
/* fprintf() from inside the callback. Drained once per loop tick.     */
/* ------------------------------------------------------------------ */

#define EVT_RING_SIZE 128

typedef struct {
    uint8_t  reportId;      /* == sh2 sensorId for sensor input reports */
    uint8_t  eventLen;      /* sh2_SensorEvent_t.len (CEVA static table) */
    uint8_t  eventSeq;      /* report[1]: per-sensor sequence byte       */
    uint16_t shtpPktLen;    /* wire length of the packet carrying it     */
    uint8_t  shtpChan;      /* SHTP channel of that packet               */
    uint8_t  shtpCont;      /* continuation flag of that packet          */
    uint64_t timestamp_uS;  /* sh2_SensorEvent_t.timestamp_uS            */
} EvtRec_t;

static EvtRec_t  sEvtRing[EVT_RING_SIZE];
static unsigned  sEvtHead = 0;
static unsigned  sEvtTail = 0;
static unsigned  sRingDrops = 0;

/* ------------------------------------------------------------------ */
/* Callbacks                                                           */
/* ------------------------------------------------------------------ */

static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    (void)cookie;

    unsigned next = (sEvtHead + 1) % EVT_RING_SIZE;
    if (next == sEvtTail) {
        /* Ring full: drop the oldest entry rather than block. */
        sEvtTail = (sEvtTail + 1) % EVT_RING_SIZE;
        sRingDrops++;
    }

    EvtRec_t *r = &sEvtRing[sEvtHead];
    r->reportId     = pEvent->reportId;
    r->eventLen     = pEvent->len;
    r->eventSeq     = (pEvent->len >= 2) ? pEvent->report[1] : 0;
    r->shtpPktLen   = sLastPktValid ? sLastPktLen : 0;
    r->shtpChan     = sLastPktValid ? sLastPktChan : 0xFF;
    r->shtpCont     = (sLastPktValid && sLastPktCont) ? 1 : 0;
    r->timestamp_uS = pEvent->timestamp_uS;

    sEvtHead = next;
}

static void asyncEventCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    if (pEvent->eventId == SH2_RESET) {
        sResetCount++;
        fprintf(stderr,
                "WARN: SH2_RESET observed (count=%u) -- data around this point may be invalid\n",
                sResetCount);
    }
    else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
        /* One per sh2_setSensorConfig() (enable or disable). */
        sGetFeatureRespCount++;
    }
}

/* ------------------------------------------------------------------ */
/* Ring drain: update stats and write CSV rows (when logging)          */
/* ------------------------------------------------------------------ */

static void drainRing(bool log)
{
    while (sEvtTail != sEvtHead) {
        const EvtRec_t *r = &sEvtRing[sEvtTail];

        if (log) {
            sRunEvents++;
            if (r->reportId == sActiveSensorId) {
                sRunMatch++;
                noteDistinct(&sRunEvtLens, r->eventLen);
                if (r->shtpChan == SHTP_CHAN_SENSOR_INPUT) {
                    noteDistinct(&sRunPktLens, r->shtpPktLen);
                }
                for (unsigned s = 0; s < NUM_TEST_SENSORS; s++) {
                    if (TEST_SENSORS[s].id == r->reportId) {
                        noteDistinct(&sSensorEvtLens[s], r->eventLen);
                        if (r->shtpChan == SHTP_CHAN_SENSOR_INPUT) {
                            noteDistinct(&sSensorPktLens[s], r->shtpPktLen);
                        }
                        sSensorEvents[s]++;
                        break;
                    }
                }
            }
            else {
                /* Stray report from the previously configured sensor, or
                 * an unexpected report type -- logged with its own
                 * report_id so it self-identifies in the CSV. */
                sRunOther++;
            }

            sCsvRowIdx++;
            fprintf(sCsv,
                    "%u,%s,%u,0x%02X,%u,%u,%u,%u,%u,%" PRIu64 "\n",
                    sCsvRowIdx,
                    sActiveSensorLabel,
                    sActiveRateHz,
                    r->reportId,
                    r->eventLen,
                    r->eventSeq,
                    r->shtpPktLen,
                    r->shtpChan,
                    r->shtpCont,
                    r->timestamp_uS);
        }

        sEvtTail = (sEvtTail + 1) % EVT_RING_SIZE;
    }
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint64_t nowUs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000ull) + ((uint64_t)ts.tv_nsec / 1000ull);
}

static int disableSensor(sh2_SensorId_t id)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = 0;   /* 0 disables the periodic report */
    return sh2_setSensorConfig(id, &cfg);
}

static int enableSensor(sh2_SensorId_t id, unsigned rateHz)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = 1000000u / rateHz;
    cfg.batchInterval_us = 0;
    return sh2_setSensorConfig(id, &cfg);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    sh2_Hal_t *hal = sh2_hal_rpi_init();
    if (hal == NULL) {
        fprintf(stderr, "FAIL: sh2_hal_rpi_init() returned NULL\n");
        return 1;
    }
    sRealHal = hal;

    /* sh2_open() gets the SHIM, never the real HAL, so every read()
     * passes through shimRead() and gets its SHTP header captured. */
    sShimHal.open       = shimOpen;
    sShimHal.close      = shimClose;
    sShimHal.read       = shimRead;
    sShimHal.write      = shimWrite;
    sShimHal.getTimeUs  = shimGetTimeUs;

    int rc = sh2_open(&sShimHal, asyncEventCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_open() returned %d\n", rc);
        return 1;
    }

    rc = sh2_setSensorCallback(sensorCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_setSensorCallback() returned %d\n", rc);
        sh2_close();
        return 1;
    }

    sCsv = fopen(REPORT_LEN_CSV, "w");
    if (sCsv == NULL) {
        fprintf(stderr, "FAIL: cannot open %s for writing\n", REPORT_LEN_CSV);
        sh2_close();
        return 1;
    }
    fprintf(sCsv,
            "idx,sensor,rate_hz,report_id,event_len,event_seq,"
            "shtp_pkt_len,shtp_chan,shtp_cont,timestamp_us\n");

    if (StartRT(RT_PRIORITY, LOOP_DT_SEC) != 0) {
        fprintf(stderr, "WARNING: StartRT() failed, continuing at default scheduling\n");
    }

    printf("== Report payload-length verification: %zu sensor(s) x %zu rate(s), %d sec each ==\n",
           NUM_TEST_SENSORS, NUM_TEST_RATES, TEST_DURATION_SEC);
    printf("Expected per CEVA sh2.c sh2ReportLens[] (+ 4-byte SHTP header for the wire packet):\n");
    for (unsigned s = 0; s < NUM_TEST_SENSORS; s++) {
        printf("  %-22s id=0x%02X  event.len=%2u  shtp_pkt=%2u\n",
               TEST_SENSORS[s].label, TEST_SENSORS[s].id,
               TEST_SENSORS[s].expectedEvtLen, TEST_SENSORS[s].expectedPktLen);
    }
    printf("\n");

    uint8_t lastConfiguredId = 0xFF;
    char b1[128];
    char b2[128];

    for (unsigned s = 0; s < NUM_TEST_SENSORS; s++) {
        const SensorSpec_t *spec = &TEST_SENSORS[s];

        for (unsigned r = 0; r < NUM_TEST_RATES; r++) {
            unsigned rate = TEST_RATES_HZ[r];

            /* When switching sensor type, disable the previous one so
             * its reports do not pollute this run. Any that still slip
             * through are counted as "other" and tagged in the CSV by
             * their own report_id. */
            if ((lastConfiguredId != 0xFF) && (lastConfiguredId != spec->id)) {
                rc = disableSensor(lastConfiguredId);
                if (rc != SH2_OK) {
                    fprintf(stderr, "WARN: disableSensor(0x%02X) returned %d\n",
                            lastConfiguredId, rc);
                }
            }

            rc = enableSensor(spec->id, rate);
            if (rc != SH2_OK) {
                fprintf(stderr, "WARN: sh2_setSensorConfig(%s @ %u Hz) returned %d, skipping\n",
                        spec->label, rate, rc);
                lastConfiguredId = spec->id;
                continue;
            }
            lastConfiguredId = spec->id;
            sActiveSensorId    = spec->id;
            sActiveSensorLabel = spec->label;
            sActiveRateHz      = rate;

            /* Settle: drain config responses / stale reports without
             * logging them. */
            uint64_t settleEnd = nowUs() + ((uint64_t)SETTLE_DELAY_SEC * 1000000ull);
            while (nowUs() < settleEnd) {
                sh2_service();
                drainRing(false);
                RT_SleepUntil(LOOP_DT_SEC);
            }
            resetRunStats();

            /* Timed window. */
            uint64_t winStart = nowUs();
            while ((nowUs() - winStart) < ((uint64_t)TEST_DURATION_SEC * 1000000ull)) {
                sh2_service();
                drainRing(true);
                RT_SleepUntil(LOOP_DT_SEC);
            }

            printf("[RUN] %-22s @ %3u Hz: %5u events (%u match, %u other) | evt.len: %s | pkt(ch3): %s | reads: %u empty / %u data | ch:",
                   spec->label, rate, sRunEvents, sRunMatch, sRunOther,
                   distinctStr(&sRunEvtLens, b1, sizeof(b1)),
                   distinctStr(&sRunPktLens, b2, sizeof(b2)),
                   sRunEmptyReads, sRunDataReads);
            for (unsigned c = 0; c < 8; c++) {
                if (sRunChanCounts[c] != 0) {
                    printf(" %u:%u", c, sRunChanCounts[c]);
                }
            }
            printf("\n");
            fflush(stdout);
        }
    }

    /* ---- Final summary ---- */
    printf("\n== Summary: measured vs expected ==\n");
    for (unsigned s = 0; s < NUM_TEST_SENSORS; s++) {
        const SensorSpec_t *spec = &TEST_SENSORS[s];
        printf("%-22s id=0x%02X  evt.len: %-20s pkt(ch3): %-20s [expected %u / %u] (%u events)\n",
               spec->label, spec->id,
               distinctStr(&sSensorEvtLens[s], b1, sizeof(b1)),
               distinctStr(&sSensorPktLens[s], b2, sizeof(b2)),
               spec->expectedEvtLen, spec->expectedPktLen,
               sSensorEvents[s]);
    }

    printf("\n== Two-phase CS-read projection ==\n");
    printf("Current rpi_read() always transfers %d bytes (SH2_HAL_MAX_TRANSFER_IN).\n",
           SH2_HAL_MAX_TRANSFER_IN);
    printf("A two-phase read (4-byte header phase + payload phase) transfers the wire pkt length:\n");
    for (unsigned s = 0; s < NUM_TEST_SENSORS; s++) {
        const SensorSpec_t *spec = &TEST_SENSORS[s];
        if (sSensorEvents[s] == 0) {
            printf("%-22s: no events, skipped\n", spec->label);
            continue;
        }
        unsigned pkt = modeOf(&sSensorPktLens[s]);
        printf("%-22s: %u bytes/data-read -> ~%.0f us at %.1f us/byte "
               "(vs ~%.0f us for the fixed %d-byte read)\n",
               spec->label, pkt,
               (double)pkt * MEASURED_US_PER_BYTE, MEASURED_US_PER_BYTE,
               (double)SH2_HAL_MAX_TRANSFER_IN * MEASURED_US_PER_BYTE,
               SH2_HAL_MAX_TRANSFER_IN);
    }

    printf("\nasync events: %u SH2_RESET, %u GET_FEATURE_RESP\n",
           sResetCount, sGetFeatureRespCount);
    if (sRingDrops != 0) {
        printf("WARN: %u ring-buffer drops (events lost from CSV)\n", sRingDrops);
    }

    fclose(sCsv);
    printf("CSV written to %s (%u event rows)\n", REPORT_LEN_CSV, sCsvRowIdx);

    sh2_close();
    printf("== Done ==\n");
    return 0;
}
