/* bno/tests/test_imu_cli.c */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cli.h"

static int failures;

static void check(bool condition, const char *caseId, const char *what)
{
    if (!condition) {
        fprintf(stderr, "FAIL %s: %s\n", caseId, what);
        ++failures;
    }
}

#define PARSE(args, result) \
    imu_cli_parse((int)(sizeof(args) / sizeof((args)[0])), (args), &(result))

static void expect_ok(const char *caseId, int argc, char *const argv[],
                      ImuCmdIdentity_t slot1, ImuCmdIdentity_t slot2,
                      ImuCmdIdentity_t slot3, bool maskPresent,
                      uint8_t mask, uint32_t duration)
{
    ImuCliParse_t parsed;

    memset(&parsed, 0xA5, sizeof(parsed));
    check(imu_cli_parse(argc, argv, &parsed) == IMU_CLI_STATUS_OK,
          caseId, "parse succeeds");
    check(parsed.status == IMU_CLI_STATUS_OK &&
          parsed.error == IMU_CLI_ERROR_NONE &&
          parsed.errorIndex == -1, caseId, "clean success status");
    check(parsed.plan.version == IMU_CMD_PLAN_VERSION,
          caseId, "plan version");
    check(parsed.plan.slot1 == slot1 &&
          parsed.plan.slot2 == slot2 &&
          parsed.plan.slot3 == slot3, caseId, "fixed slot order");
    check(parsed.plan.probeMaskPresent == maskPresent,
          caseId, "probe mask presence");
    check(parsed.plan.probeMask == mask, caseId, "probe mask value");
    check(parsed.plan.acquisitionDurationS == duration,
          caseId, "acquisition-only duration");
    check(parsed.plan.flightCalMask == 0u &&
          parsed.plan.probeDeadlineS == IMU_CMD_PROBE_DEADLINE_S &&
          parsed.plan.settleDurationMs == IMU_CMD_SETTLE_DEFAULT_MS &&
          parsed.plan.servicePeriodNs == IMU_CMD_SERVICE_PERIOD_NS &&
          parsed.plan.publicationPeriodNs == IMU_CMD_PUB_PERIOD_NS,
          caseId, "frozen defaults");
    check(imu_cmd_init(&parsed.plan), caseId,
          "direct coordinator accepts parser plan");
}

static void expect_error(const char *caseId, int argc,
                         char *const argv[], ImuCliError_t error,
                         int index)
{
    ImuCliParse_t parsed;
    ImuCmdPlan_t zeroPlan;

    memset(&parsed, 0xA5, sizeof(parsed));
    memset(&zeroPlan, 0, sizeof(zeroPlan));
    check(imu_cli_parse(argc, argv, &parsed) == IMU_CLI_STATUS_ERROR,
          caseId, "parse rejects");
    check(parsed.status == IMU_CLI_STATUS_ERROR &&
          parsed.error == error && parsed.errorIndex == index,
          caseId, "error category and index");
    check(memcmp(&parsed.plan, &zeroPlan, sizeof(zeroPlan)) == 0,
          caseId, "no partially usable plan");
}

static void test_accepted_forms(void)
{
    char *none[] = {"bno_app"};
    char *cal[] = {"bno_app", "--cal-imu"};
    char *dcd[] = {"bno_app", "--cal-imu", "--clear"};
    char *tare[] = {"bno_app", "--tare-imu"};
    char *full[] = {"bno_app", "--tare-imu", "--full"};
    char *tcheck[] = {"bno_app", "--tare-imu", "--check"};
    char *tclear[] = {"bno_app", "--tare-imu", "--clear"};
    char *checkOnly[] = {"bno_app", "--check-imu"};
    char *probeZero[] = {"bno_app", "--check-imu", "--mask", "0"};
    char *probeHexZero[] = {"bno_app", "--check-imu", "--mask", "0x00"};
    char *probeMax[] = {"bno_app", "--check-imu", "--mask", "255"};
    char *probeHexMax[] = {"bno_app", "--check-imu", "--mask", "0XFF"};
    char *minDuration[] = {"bno_app", "--duration", "1"};
    char *maxDuration[] = {"bno_app", "--duration", "3600"};
    char *plusDuration[] = {"bno_app", "--duration", "+10"};
    char *combined[] = {"bno_app", "--mask", "0x05", "--check-imu",
                        "--tare-imu", "--full", "--cal-imu",
                        "--duration", "2"};
    char *reordered[] = {"bno_app", "--duration", "2", "--cal-imu",
                         "--tare-imu", "--check-imu", "--mask", "5",
                         "--full"};
    ImuCliParse_t p;

    expect_ok("CLI01", 1, none, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI02", 2, cal, IMU_CMD_ID_CALIBRATION, IMU_CMD_ID_NONE,
              IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI03", 3, dcd, IMU_CMD_ID_DCD_CLEAR, IMU_CMD_ID_NONE,
              IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI04", 2, tare, IMU_CMD_ID_NONE, IMU_CMD_ID_TARE,
              IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI05", 3, full, IMU_CMD_ID_NONE, IMU_CMD_ID_TARE,
              IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI06", 3, tcheck, IMU_CMD_ID_NONE,
              IMU_CMD_ID_TARE_CHECK, IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI07", 3, tclear, IMU_CMD_ID_NONE,
              IMU_CMD_ID_TARE_CLEAR, IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI08", 2, checkOnly, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_CHECK, false, 0u, 10u);
    expect_ok("CLI09", 4, probeZero, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_PROBE, true, 0u, 10u);
    expect_ok("CLI10", 4, probeHexZero, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_PROBE, true, 0u, 10u);
    expect_ok("CLI11", 4, probeMax, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_PROBE, true, 255u, 10u);
    expect_ok("CLI12", 4, probeHexMax, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_PROBE, true, 255u, 10u);
    expect_ok("CLI13", 3, minDuration, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_NONE, false, 0u, 1u);
    expect_ok("CLI14", 3, maxDuration, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_NONE, false, 0u, 3600u);
    expect_ok("CLI15", 3, plusDuration, IMU_CMD_ID_NONE, IMU_CMD_ID_NONE,
              IMU_CMD_ID_NONE, false, 0u, 10u);
    expect_ok("CLI16", 9, combined, IMU_CMD_ID_CALIBRATION,
              IMU_CMD_ID_TARE, IMU_CMD_ID_PROBE, true, 5u, 2u);
    expect_ok("CLI17", 9, reordered, IMU_CMD_ID_CALIBRATION,
              IMU_CMD_ID_TARE, IMU_CMD_ID_PROBE, true, 5u, 2u);

    check(PARSE(dcd, p) == IMU_CLI_STATUS_OK &&
          p.plan.confirmDcdClear && !p.plan.confirmTareClear,
          "CLI18", "DCD confirmation belongs to slot 1");
    check(PARSE(tclear, p) == IMU_CLI_STATUS_OK &&
          p.plan.confirmTareClear && !p.plan.confirmDcdClear,
          "CLI19", "tare-clear confirmation belongs to slot 2");
    check(PARSE(tare, p) == IMU_CLI_STATUS_OK &&
          p.plan.tareAxes == IMU_CMD_TARE_AXES_Z &&
          p.plan.persistTare && p.plan.confirmTare,
          "CLI20", "Z tare persists and confirms");
    check(PARSE(full, p) == IMU_CLI_STATUS_OK &&
          p.plan.tareAxes == IMU_CMD_TARE_AXES_FULL &&
          p.plan.persistTare && p.plan.confirmTare,
          "CLI21", "full tare persists and confirms");
    check(PARSE(tcheck, p) == IMU_CLI_STATUS_OK &&
          !p.plan.persistTare, "CLI22", "tare check is not tare-now");
}

static void test_invalid_forms(void)
{
    char *unknown[] = {"bno_app", "--cal"};
    char *positional[] = {"bno_app", "unexpected"};
    char *missingMask[] = {"bno_app", "--check-imu", "--mask"};
    char *nextOption[] = {"bno_app", "--check-imu", "--mask", "--full"};
    char *missingDuration[] = {"bno_app", "--duration"};
    char *duplicateCal[] = {"bno_app", "--cal-imu", "--cal-imu"};
    char *duplicateMask[] = {"bno_app", "--check-imu", "--mask", "1",
                             "--mask", "2"};
    char *duplicateClear[] = {"bno_app", "--cal-imu", "--clear", "--clear"};
    char *orphanClear[] = {"bno_app", "--clear"};
    char *ambiguousClear[] = {"bno_app", "--cal-imu", "--tare-imu",
                              "--clear"};
    char *orphanFull[] = {"bno_app", "--full"};
    char *orphanTcheck[] = {"bno_app", "--check"};
    char *orphanMask[] = {"bno_app", "--mask", "0"};
    char *fullCheck[] = {"bno_app", "--tare-imu", "--full", "--check"};
    char *fullClear[] = {"bno_app", "--tare-imu", "--full", "--clear"};
    char *checkClear[] = {"bno_app", "--tare-imu", "--check", "--clear"};
    char *help[] = {"bno_app", "--help"};
    char *helpOther[] = {"bno_app", "--help", "--duration", "1"};
    ImuCliParse_t parsed;
    ImuCmdPlan_t zeroPlan;

    expect_error("CLI30", 2, unknown, IMU_CLI_ERROR_UNKNOWN_OPTION, 1);
    expect_error("CLI31", 2, positional, IMU_CLI_ERROR_POSITIONAL, 1);
    expect_error("CLI32", 3, missingMask, IMU_CLI_ERROR_MISSING_VALUE, 2);
    expect_error("CLI33", 4, nextOption, IMU_CLI_ERROR_MISSING_VALUE, 2);
    expect_error("CLI34", 2, missingDuration,
                 IMU_CLI_ERROR_MISSING_VALUE, 1);
    expect_error("CLI35", 3, duplicateCal,
                 IMU_CLI_ERROR_REPEATED_OPTION, 2);
    expect_error("CLI36", 6, duplicateMask,
                 IMU_CLI_ERROR_REPEATED_OPTION, 4);
    expect_error("CLI37", 4, duplicateClear,
                 IMU_CLI_ERROR_REPEATED_OPTION, 3);
    expect_error("CLI38", 2, orphanClear,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 1);
    expect_error("CLI39", 4, ambiguousClear,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 3);
    expect_error("CLI40", 2, orphanFull,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 1);
    expect_error("CLI41", 2, orphanTcheck,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 1);
    expect_error("CLI42", 3, orphanMask,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 1);
    expect_error("CLI43", 4, fullCheck,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 2);
    expect_error("CLI44", 4, fullClear,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 2);
    expect_error("CLI45", 4, checkClear,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 2);
    expect_error("CLI46", 4, helpOther,
                 IMU_CLI_ERROR_ILLEGAL_COMBINATION, 1);

    memset(&parsed, 0xA5, sizeof(parsed));
    memset(&zeroPlan, 0, sizeof(zeroPlan));
    check(PARSE(help, parsed) == IMU_CLI_STATUS_HELP &&
          parsed.status == IMU_CLI_STATUS_HELP &&
          parsed.error == IMU_CLI_ERROR_NONE &&
          memcmp(&parsed.plan, &zeroPlan, sizeof(zeroPlan)) == 0,
          "CLI47", "exclusive help, no usable plan");
}

static void test_mask_lexing(void)
{
    char *hexWithoutDigits[] = {"bno_app", "--check-imu", "--mask", "0x"};
    char *negative[] = {"bno_app", "--check-imu", "--mask", "-1"};
    char *plus[] = {"bno_app", "--check-imu", "--mask", "+1"};
    char *junk[] = {"bno_app", "--check-imu", "--mask", "0x1g"};
    char *whitespace[] = {"bno_app", "--check-imu", "--mask", " 1"};
    char *tooLargeDec[] = {"bno_app", "--check-imu", "--mask", "256"};
    char *tooLargeHex[] = {"bno_app", "--check-imu", "--mask", "0x100"};
    char *huge[] = {"bno_app", "--check-imu", "--mask",
                    "999999999999999999999999999999999"};
    char *leadingZero[] = {"bno_app", "--check-imu", "--mask", "005"};
    ImuCliParse_t parsed;

    expect_error("CLI50", 4, hexWithoutDigits,
                 IMU_CLI_ERROR_INVALID_VALUE, 3);
    expect_error("CLI51", 4, negative, IMU_CLI_ERROR_INVALID_VALUE, 3);
    expect_error("CLI52", 4, plus, IMU_CLI_ERROR_INVALID_VALUE, 3);
    expect_error("CLI53", 4, junk, IMU_CLI_ERROR_INVALID_VALUE, 3);
    expect_error("CLI54", 4, whitespace, IMU_CLI_ERROR_INVALID_VALUE, 3);
    expect_error("CLI55", 4, tooLargeDec, IMU_CLI_ERROR_OUT_OF_RANGE, 3);
    expect_error("CLI56", 4, tooLargeHex, IMU_CLI_ERROR_OUT_OF_RANGE, 3);
    expect_error("CLI57", 4, huge, IMU_CLI_ERROR_OUT_OF_RANGE, 3);
    check(PARSE(leadingZero, parsed) == IMU_CLI_STATUS_OK &&
          parsed.plan.probeMask == 5u &&
          parsed.plan.probeMaskPresent,
          "CLI58", "leading-zero token parsed as decimal");
}

static void test_duration_lexing(void)
{
    char *zero[] = {"bno_app", "--duration", "0"};
    char *negative[] = {"bno_app", "--duration", "-1"};
    char *decimal[] = {"bno_app", "--duration", "10.5"};
    char *suffix[] = {"bno_app", "--duration", "10s"};
    char *hex[] = {"bno_app", "--duration", "0x0A"};
    char *plusOnly[] = {"bno_app", "--duration", "+"};
    char *tooLarge[] = {"bno_app", "--duration", "3601"};
    char *huge[] = {"bno_app", "--duration",
                    "999999999999999999999999999999999"};

    expect_error("CLI60", 3, zero, IMU_CLI_ERROR_OUT_OF_RANGE, 2);
    expect_error("CLI61", 3, negative, IMU_CLI_ERROR_INVALID_VALUE, 2);
    expect_error("CLI62", 3, decimal, IMU_CLI_ERROR_INVALID_VALUE, 2);
    expect_error("CLI63", 3, suffix, IMU_CLI_ERROR_INVALID_VALUE, 2);
    expect_error("CLI64", 3, hex, IMU_CLI_ERROR_INVALID_VALUE, 2);
    expect_error("CLI65", 3, plusOnly, IMU_CLI_ERROR_INVALID_VALUE, 2);
    expect_error("CLI66", 3, tooLarge, IMU_CLI_ERROR_OUT_OF_RANGE, 2);
    expect_error("CLI67", 3, huge, IMU_CLI_ERROR_OUT_OF_RANGE, 2);
}

static void test_direct_plan_validation(void)
{
    ImuCmdPlan_t plan;

    imu_cmd_plan_clear(&plan);
    plan.confirmDcdClear = true;
    check(!imu_cmd_init(&plan), "CLI70", "orphan clear confirm rejected");
    check(imu_cmd_init_reason() == IMU_CMD_REASON_ILLEGAL_PLAN,
          "CLI70", "coordinator reason");

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE_CHECK;
    plan.persistTare = true;
    check(!imu_cmd_init(&plan), "CLI71", "orphan persist rejected");

    imu_cmd_plan_clear(&plan);
    plan.slot3 = IMU_CMD_ID_CHECK;
    plan.probeMaskPresent = true;
    plan.probeMask = 5u;
    check(!imu_cmd_init(&plan), "CLI72", "present mask on CHECK rejected");
}

int main(void)
{
    test_accepted_forms();
    test_invalid_forms();
    test_mask_lexing();
    test_duration_lexing();
    test_direct_plan_validation();

    if (failures != 0) {
        fprintf(stderr, "test_imu_cli: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_imu_cli: pass\n");
    return 0;
}
