#!/bin/sh
# PC tests for subsystem 4 pure code (imu_hump.c, imu_calib.c):
#  1. test_hump.c: simulated humps (derived)
#  2. replay_trace.c: the real car trace of run 45 (10 mm ramp under the
#     right wheel, measured 2026-11-04) must give exactly 1 hump, 8-12 mm
#  3. test_lift.c: lift / overturn detection (imu_lift.c), simulated
#  4. replay_lift.c: real car [LT] traces (2026-11-16 runs 03-08): the
#     hand lifts must fire LIFTED, the tip-over OVERTURNED; the drive-off
#     jolt, flat knock and bang must fire nothing
set -eu
cd "$(dirname "$0")"
bin=$(mktemp)
rep=$(mktemp)
lft=$(mktemp)
rlf=$(mktemp)
trap 'rm -f "$bin" "$rep" "$lft" "$rlf"' EXIT
S4=../../../app_program/subsystem4
LOGS=../logs
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   test_hump.c $S4/imu_hump.c $S4/imu_calib.c -o "$bin" -lm
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   replay_trace.c $S4/imu_hump.c $S4/imu_calib.c -o "$rep" -lm
"$bin"
"$rep" $LOGS/2026-11-04_hump_trace_run45.txt 8 12 1
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   test_lift.c $S4/imu_lift.c -o "$lft" -lm
"$lft"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   replay_lift.c $S4/imu_lift.c -o "$rlf"
"$rlf" $LOGS/2026-11-16_lift_run03_handlift_hump25.txt L
"$rlf" $LOGS/2026-11-16_lift_run04_false_jolt_knock_hump21.txt NN
"$rlf" $LOGS/2026-11-16_lift_run06_2lifts_1false_hump27.txt LLN
"$rlf" $LOGS/2026-11-16_lift_run07_veto_3lifts_hump25_27.txt LLL
"$rlf" $LOGS/2026-11-16_lift_run08_lift_overturn.txt LO
# run 11 (line follower + 4 hand lifts) replayed with the vz column, the
# firmware input since 2026-11-16 (runs 03-08 have no vz: dev input)
"$rlf" $LOGS/2026-11-16_lift_run11_line_follow_vz_lifts.txt NNNNLLLNNLN vz
"$rlf" $LOGS/2026-11-17_lift_run12_vz_line_follow_lift.txt LN vz
"$rlf" $LOGS/2026-11-17_lift_run13_U41_gp20_while_lifted.txt L vz
