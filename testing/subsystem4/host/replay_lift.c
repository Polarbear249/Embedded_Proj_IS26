/*
 * PC replay of real car lift traces through the production imu_lift.c
 * (regression test; MEASURED data, not simulated).
 *
 * Reads the "[LT] k d dev v p r c" blocks that an IMU_LIFT_TRACE build
 * prints after every LIFTED / OVERTURNED event, feeds each block's
 * samples (dev, pitch, roll; 10 ms each) to a fresh imu_lift_t and
 * reports whether LIFTED fires, at which k, and the peak estimated
 * upward speed. The firmware's own state before the block is not in the
 * log, so each block starts from zero (the logs show |v| < 35 mm/s at
 * the start of every block, so this is a small difference).
 *
 * Usage: replay_lift <log file> <expect> [vz]
 *   vz: feed the 8th column (a along gravity, traces from 2026-11-16 run 11
 *   on) to the detector instead of dev (|a| - g_ref). Blocks without it
 *   fall back to dev.
 *   expect: one letter per [LT] block, in order: L = must fire LIFTED,
 *   O = must fire OVERTURNED,
 *   N = must not fire. Example: replay_lift run04.txt NN
 * Exit 0 when every block matches.
 */
#include <stdio.h>
#include <string.h>

#include "../../../app_program/subsystem4/imu_lift.h"

#define LINE_MAX 256

int main(int argc, char **argv)
{
    FILE      *p_file = NULL;
    char       line[LINE_MAX];
    imu_lift_t lift;
    int        in_block = 0;
    int        block = 0;
    int        fired_k = 0;
    int        fired = 0;
    int        peak = 0;
    int        fails = 0;
    size_t     expect_len = 0U;

    int        use_vz = 0;

    if ((argc != 3) && (argc != 4)) {
        fprintf(stderr, "usage: %s <log> <expect: L/O/N per block>\n", argv[0]);
        return 2;
    }
    p_file = fopen(argv[1], "r");
    if (p_file == NULL) {
        perror(argv[1]);
        return 2;
    }
    expect_len = strlen(argv[2]);
    use_vz = (argc == 4) && (strcmp(argv[3], "vz") == 0);

    while (fgets(line, sizeof(line), p_file) != NULL) {
        int k, d, dev, v, p, r, c, vz = 0;
        int n = 0;

        if (strstr(line, "[LT] begin") != NULL) {
            imu_lift_init(&lift);
            in_block = 1;
            fired = 0;
            fired_k = 0;
            peak = 0;
        } else if (in_block && (strstr(line, "[LT] end") != NULL)) {
            char want = (block < (int)expect_len) ? argv[2][block] : '?';
            char got = (fired == 0) ? 'N' : (char)fired;
            int  ok = (want == got);

            printf("block %d: %-10s at k %4d, peak %4d mm/s  (want %c)  %s\n",
                   block + 1,
                   (got == 'L') ? "LIFTED" : ((got == 'O') ? "OVERTURNED"
                                                           : "none"),
                   fired ? fired_k : 0,
                   peak, want, ok ? "PASS" : "FAIL");
            fails += ok ? 0 : 1;
            block++;
            in_block = 0;
        } else if (in_block && ((n = sscanf(line, "[LT] %d %d %d %d %d %d %d %d",
                                            &k, &d, &dev, &v, &p, &r, &c,
                                            &vz)) >= 7)) {
            int in = (use_vz && (n == 8)) ? vz : dev;
            imu_lift_result_t res = imu_lift_update(&lift, in, p, r);
            int vel = imu_lift_vel_mm_s(lift.vel_sum);

            peak = (vel > peak) ? vel : peak;
            if ((IMU_LIFT_LIFTED == res) && !fired) {
                fired = 'L';
                fired_k = k;
            } else if ((IMU_LIFT_OVERTURNED == res) && !fired) {
                fired = 'O';
                fired_k = k;
            }
        }
    }
    fclose(p_file);

    if (block != (int)expect_len) {
        printf("expected %u blocks, found %d  FAIL\n",
               (unsigned)expect_len, block);
        fails++;
    }
    printf("%s: %s, clamp %d mg, threshold %d mm/s -> %s\n", argv[1],
           use_vz ? "vz" : "dev", IMU_LIFT_CLAMP_MG, IMU_LIFT_VEL_MM_S,
           fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
