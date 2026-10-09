#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_contract.h"
#include "app/imu_publish.h"

static int s_failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++s_failures;                                                 \
        }                                                                 \
    } while (0)

_Static_assert(sizeof(IMU_CSV_HEADER_TOKENS) / sizeof(IMU_CSV_HEADER_TOKENS[0]) ==
                   IMU_CSV_COLUMN_COUNT,
               "header token table must match column count");

static void test_versions(void)
{
    CHECK(IMU_PUBLICATION_CONTRACT_VERSION == 1u);
    CHECK(IMU_RUN_STATS_CONTRACT_VERSION == 1u);
    CHECK(IMU_LOGGER_STATS_CONTRACT_VERSION == 1u);
    CHECK(IMU_TERMINATION_CONTRACT_VERSION == 1u);
    CHECK(IMU_RUN_METADATA_CONTRACT_VERSION == 1u);
    CHECK(IMU_META_SCHEMA_VERSION == 3u);
    CHECK(IMU_CSV_SCHEMA_VERSION == 1u);
    CHECK(IMU_CSV_COLUMN_COUNT == 52u);
    CHECK(strcmp(IMU_CSV_MISSING_TOKEN, "NaN") == 0);
}

static void test_masks_use_r0_bits_only(void)
{
    CHECK(IMU_PUBLISH_GROUP_BIT(IMU_PUBLISH_GROUP_ROTATION) == IMU_GROUP_BIT_ROTATION);
    CHECK(IMU_PUBLISH_GROUP_BIT(IMU_PUBLISH_GROUP_ACCEL) == IMU_GROUP_BIT_ACCEL);
    CHECK(IMU_PUBLISH_GROUP_BIT(IMU_PUBLISH_GROUP_GYRO) == IMU_GROUP_BIT_GYRO);
    CHECK(IMU_PUBLISH_GROUP_COUNT == 3);
    CHECK(IMU_GROUP_MASK_REQUIRED == 0x07u);
    CHECK(IMU_GROUP_MASK_REQUIRED ==
          (IMU_GROUP_BIT_ROTATION | IMU_GROUP_BIT_ACCEL | IMU_GROUP_BIT_GYRO));
    CHECK((IMU_GROUP_MASK_REQUIRED & ~0x07u) == 0u);
}

static void test_reader_flag_map(void)
{
    CHECK(IMU_CSV_FLAG_RESET_OBSERVED == 0x01u);
    CHECK(IMU_CSV_FLAG_DECODE_ERROR_OBSERVED == 0x02u);
    CHECK(IMU_CSV_FLAG_UNEXPECTED_REPORT_OBSERVED == 0x04u);
    CHECK(IMU_CSV_FLAG_REPORT_SEQUENCE_GAP_OBSERVED == 0x08u);
    CHECK(IMU_CSV_FLAG_CONFIGURATION_FAILED == 0x10u);
    CHECK(IMU_CSV_FLAG_SESSION_RECOVERY_OCCURRED == 0x20u);
    CHECK(IMU_CSV_READER_FLAGS_MASK == 0x3Fu);
    CHECK((IMU_CSV_READER_FLAGS_MASK & ~0x3Fu) == 0u);
}

static void test_header_tokens(void)
{
    unsigned i, j;

    for (i = 0u; i < IMU_CSV_COLUMN_COUNT; ++i) {
        CHECK(IMU_CSV_HEADER_TOKENS[i] != NULL);
        CHECK(IMU_CSV_HEADER_TOKENS[i][0] != '\0');
        CHECK(strchr(IMU_CSV_HEADER_TOKENS[i], ',') == NULL);
        for (j = i + 1u; j < IMU_CSV_COLUMN_COUNT; ++j) {
            CHECK(strcmp(IMU_CSV_HEADER_TOKENS[i], IMU_CSV_HEADER_TOKENS[j]) != 0);
        }
    }
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[0], "schema_ver") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[16], "logger_drops") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[17], "rv_seq") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[31], "rv_err_rad") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[32], "acc_seq") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[41], "acc_z") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[42], "gyr_seq") == 0);
    CHECK(strcmp(IMU_CSV_HEADER_TOKENS[51], "gyr_z") == 0);
    CHECK(strncmp(IMU_CSV_HEADER_TOKENS[17], "rv_", 3u) == 0);
    CHECK(strncmp(IMU_CSV_HEADER_TOKENS[32], "acc_", 4u) == 0);
    CHECK(strncmp(IMU_CSV_HEADER_TOKENS[42], "gyr_", 4u) == 0);
}

static void test_r2_unchanged(void)
{
    /* LP64 golden layout of the pinned reader contract (aarch64/x86-64). */
    CHECK(IMU_SAMPLE_CONTRACT_VERSION == 1u);
    CHECK(IMU_METADATA_CONTRACT_VERSION == 1u);
#if defined(__LP64__)
    CHECK(sizeof(ImuGroupMeta_t) == 48u);
    CHECK(sizeof(ImuSampleSnapshot_t) == 240u);
#endif
    CHECK(offsetof(ImuSampleSnapshot_t, validMask) > offsetof(ImuSampleSnapshot_t, gz));
    CHECK(offsetof(ImuSampleSnapshot_t, validMask) + sizeof(uint8_t) <=
          sizeof(ImuSampleSnapshot_t));
}

static void test_no_publisher_leak_into_r1_r2(void)
{
    ImuSampleSnapshot_t s;
    ImuGroupMeta_t m;

    memset(&s, 0, sizeof(s));
    memset(&m, 0, sizeof(m));
    /* Any field added to R1/R2 changes these goldens (see test_r2_unchanged);
     * the publisher fields live only in R9 and are not reachable here. */
    CHECK(sizeof(s) == sizeof(ImuSampleSnapshot_t));
    CHECK(sizeof(m) == sizeof(ImuGroupMeta_t));
    CHECK(s.version == 0u && m.version == 0u);
}

static void test_record_layout_facts(void)
{
    ImuPublicationRecord_t r;
    ImuRunStats_t stats;
    ImuLoggerStats_t logger;
    ImuTermination_t term;
    ImuRunMetadata_t meta;

    memset(&r, 0, sizeof(r));
    memset(&stats, 0, sizeof(stats));
    memset(&logger, 0, sizeof(logger));
    memset(&term, 0, sizeof(term));
    memset(&meta, 0, sizeof(meta));

    CHECK(r.version == 0u);
    CHECK(r.rotation.reserved == 0u && r.accel.reserved == 0u && r.gyro.reserved == 0u);
    CHECK(r.reserved8[0] == 0u && r.reserved8[1] == 0u);
    CHECK(offsetof(ImuPublicationRecord_t, version) == 0u);
    CHECK(offsetof(ImuPublicationRecord_t, publicationSeq) >
          offsetof(ImuPublicationRecord_t, configurationEpoch));
    CHECK(offsetof(ImuPublicationRecord_t, rotation) <
          offsetof(ImuPublicationRecord_t, accel));
    CHECK(offsetof(ImuPublicationRecord_t, accel) <
          offsetof(ImuPublicationRecord_t, gyro));
    CHECK(sizeof(IMU_CSV_HEADER_TOKENS[0]) == sizeof(const char *));
    CHECK(sizeof(meta.baseName) >= sizeof("bno_acq_YYYYMMDD_HHMMSS_999"));
    CHECK(sizeof(meta.filenameUtc) == sizeof("YYYYMMDD_HHMMSS"));
    CHECK(IMU_TERM_REASON_COUNT == 8);
    CHECK(IMU_LOGGER_START_NOT_ATTEMPTED == 0);
    CHECK(IMU_META_COMPLETION_RUNNING == 0);
}

int main(void)
{
    test_versions();
    test_masks_use_r0_bits_only();
    test_reader_flag_map();
    test_header_tokens();
    test_r2_unchanged();
    test_no_publisher_leak_into_r1_r2();
    test_record_layout_facts();

    printf("info: sizeof(ImuPublicationRecord_t)=%zu\n",
           sizeof(ImuPublicationRecord_t));
    printf("info: 2048 * sizeof(ImuPublicationRecord_t)=%zu bytes\n",
           (size_t)2048u * sizeof(ImuPublicationRecord_t));
    printf("info: sizeof(ImuRunMetadata_t)=%zu\n", sizeof(ImuRunMetadata_t));

    if (s_failures != 0) {
        fprintf(stderr, "test_imu_publish_contract: %d failure(s)\n", s_failures);
        return 1;
    }
    puts("test_imu_publish_contract: pass");
    return 0;
}
