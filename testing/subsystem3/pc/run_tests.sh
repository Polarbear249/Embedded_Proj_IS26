#!/bin/sh
# Build and run the PC unit tests for Subsystem 3 (decoder, capture,
# line logic, IF-06 word). The firmware modules are compiled with the
# strict warning set; any warning fails the build. C11 = the firmware's
# gnu11 without GNU extensions.
set -e
cd "$(dirname "$0")"
SRC=../../../app_program/subsystem3
OUT=${TMPDIR:-/tmp}/s3_pc_test
gcc -std=c11 -Wall -Wextra -pedantic -Wconversion -Wshadow -Werror -O1 \
    -c $SRC/barcode_code39.c $SRC/barcode_capture.c $SRC/line_logic.c \
    $SRC/line_if.c $SRC/line_drive.c -I$SRC
gcc -std=c11 -Wall -Wextra -O1 -I$SRC -o "$OUT" test_s3.c \
    barcode_code39.o barcode_capture.o line_logic.o line_if.o line_drive.o
rm -f barcode_code39.o barcode_capture.o line_logic.o line_if.o line_drive.o
"$OUT"
