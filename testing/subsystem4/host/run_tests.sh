#!/bin/sh
# PC tests for subsystem 4 pure code (imu_hump.c, imu_calib.c):
#  1. test_hump.c: simulated humps (derived)
#  2. replay_trace.c: the real car trace of run 45 (10 mm ramp under the
#     right wheel, measured 2026-11-04) must give exactly 1 hump, 8-12 mm
set -eu
cd "$(dirname "$0")"
bin=$(mktemp)
rep=$(mktemp)
trap 'rm -f "$bin" "$rep"' EXIT
S4=../../../app_program/subsystem4
LOGS=../logs
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   test_hump.c $S4/imu_hump.c $S4/imu_calib.c -o "$bin" -lm
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   replay_trace.c $S4/imu_hump.c $S4/imu_calib.c -o "$rep" -lm
"$bin"
"$rep" $LOGS/2026-11-04_hump_trace_run45.txt 8 12 1
