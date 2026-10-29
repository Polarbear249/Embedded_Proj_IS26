#!/bin/sh
set -eu
cd "$(dirname "$0")"
bin=$(mktemp)
trap 'rm -f "$bin"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Iinclude test_motion.c -o "$bin"
"$bin"
