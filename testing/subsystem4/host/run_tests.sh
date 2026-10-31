#!/bin/sh
# PC tests for subsystem 4 pure code (imu_hump.c, imu_calib.c).
set -eu
cd "$(dirname "$0")"
bin=$(mktemp)
trap 'rm -f "$bin"' EXIT
S4=../../../app_program/subsystem4
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
   test_hump.c $S4/imu_hump.c $S4/imu_calib.c -o "$bin" -lm
"$bin"
