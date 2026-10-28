#!/bin/sh
# Build and run the PC unit tests for the Subsystem 1 frame codec.
set -e
cd "$(dirname "$0")"
gcc -std=c99 -Wall -Wextra -pedantic -Werror -O1 -o /tmp/s1_test_frame \
    test_frame.c ../../../app_program/subsystem1/telemetry_frame.c
/tmp/s1_test_frame
