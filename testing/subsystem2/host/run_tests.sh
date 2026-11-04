#!/bin/sh
set -eu
cd "$(dirname "$0")"
bin=$(mktemp)
trap 'rm -f "$bin"' EXIT
# MinGW has no libubsan: set S2_HOST_SAN= to build without the sanitizer.
san=${S2_HOST_SAN--fsanitize=undefined}
cc -std=c11 -Wall -Wextra -Werror $san -Iinclude test_encoder_distance.c -o "$bin" -lm
"$bin"
cc -std=c11 -Wall -Wextra -Werror $san test_motion_move.c -o "$bin"
"$bin"
