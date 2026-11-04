/*
 * replay_trace.c
 * PC replay of a car hump trace through the production imu_hump.c.
 * Input: a log with "[TR] d_mm pitch_0.1deg roll_0.1deg z_mm" lines, as
 * printed by imu_terrain.c with IMU_HUMP_TRACE 1 (every 40 ms while the
 * car moves). Each line is fed as one update, as the task would.
 *
 * Usage: replay_trace <log> [min_mm max_mm humps]
 *   With the three numbers it is a test: PASS when exactly <humps> humps
 *   ended and every reported height is within min..max mm.
 *
 * Real-data regression test for the hump model (run 45 = 10 mm ramp
 * under the right wheel, flat floor elsewhere).
 */
#include <stdio.h>
#include <stdlib.h>

#include "../../../app_program/subsystem4/imu_hump.h"

int main(int argc, char **argv)
{
    FILE *p_file;
    char line[256];
    imu_hump_t hump;
    int d, p, r, z;
    int starts = 0, ends = 0, samples = 0, ok = 1;
    int min_mm = 0, max_mm = 0, want = -1;
    unsigned lo = 1000U, hi = 0U;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <log> [min_mm max_mm humps]\n", argv[0]);
        return 2;
    }
    if (argc >= 5) {
        min_mm = atoi(argv[2]);
        max_mm = atoi(argv[3]);
        want = atoi(argv[4]);
    }
    p_file = fopen(argv[1], "r");
    if (NULL == p_file) {
        perror(argv[1]);
        return 2;
    }

    imu_hump_init(&hump);
    while (NULL != fgets(line, sizeof(line), p_file)) {
        if (4 != sscanf(line, "[TR] %d %d %d %d", &d, &p, &r, &z)) {
            continue;
        }
        samples++;
        switch (imu_hump_update(&hump, p, r, d)) {
        case IMU_EVT_HUMP_START:
            starts++;
            printf("  start at d %d\n", d);
            break;
        case IMU_EVT_HUMP_END:
            ends++;
            printf("  END   at d %d: height %u mm\n", d, hump.last_mm);
            lo = (hump.last_mm < lo) ? hump.last_mm : lo;
            hi = (hump.last_mm > hi) ? hump.last_mm : hi;
            break;
        default:
            break;
        }
    }
    fclose(p_file);

    printf("  AVG %d mm, DRIVEOFF %d mm, ON %d OFF %d: %d samples, "
           "starts %d, ends %d, last %u, max %u, valid %d\n",
           IMU_HUMP_AVG_MM, IMU_HUMP_DRIVEOFF_MM, IMU_HUMP_ON_DDEG,
           IMU_HUMP_OFF_DDEG, samples, starts, ends, hump.last_mm,
           hump.max_mm, (int)hump.b_valid);
    if (want >= 0) {
        ok = (ends == want) && ((0 == ends)
             || (((int)lo >= min_mm) && ((int)hi <= max_mm)));
        printf("%-34s %s\n", "replay of the car trace", ok ? "PASS" : "FAIL");
    }
    return ok ? 0 : 1;
}
