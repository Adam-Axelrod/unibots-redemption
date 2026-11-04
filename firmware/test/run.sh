#!/bin/bash
# Build and run the hardware-free tests with the laptop's own compiler.
set -euo pipefail
cd "$(dirname "$0")"
cc -std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined -I../main \
    test_core.c ../main/mecanum.c ../main/wheel_ctl.c ../main/ms200.c -lm -o run_tests
./run_tests
