#!/bin/sh
# Build and run the PC unit tests for the Vehicle Controller mission logic
# (app_program/vc_mission.c). The firmware module is compiled with the
# strict warning set; any warning fails the build.
set -e
cd "$(dirname "$0")"
SRC=../../../app_program
OUT=${TMPDIR:-/tmp}/vc_pc_test
gcc -std=c11 -Wall -Wextra -pedantic -Wconversion -Wshadow -Werror -O1 \
    -c $SRC/vc_mission.c -I$SRC -o vc_mission.o
gcc -std=c11 -Wall -Wextra -O1 -I$SRC -o "$OUT" test_vc_mission.c vc_mission.o
rm -f vc_mission.o
"$OUT"
