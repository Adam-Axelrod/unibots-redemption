#!/usr/bin/env bash
# Launch the simulator, then run every robot_control node against it.
#
# This catches what a build cannot: a node that compiles perfectly and then
# crashes on its first /odom message, or hangs on Ctrl-C and leaves the real
# robot driving. Each node must
#
#     1. still be alive after RUN_SECONDS, and
#     2. shut down cleanly when interrupted.
#
# It also checks the simulator itself is publishing, and that square_driver
# actually moves the robot -- a real end-to-end pass through /cmd_vel.
#
# Run it locally the same way CI does:
#     docker compose run --rm --entrypoint bash sim -c \
#       'colcon build --symlink-install && .github/scripts/smoke_test.sh'
# No `set -u`: ROS's own setup.bash reads unset variables and would abort the script.
# No `set -e` either -- a failing check must be recorded and reported, not exit early.
set -o pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT/ws"

RUN_SECONDS="${RUN_SECONDS:-8}"
LOGDIR="$(mktemp -d)"
SIM_PID=""
failures=0

source /opt/ros/humble/setup.bash
source install/setup.bash

note() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
pass() { printf '    \033[32mPASS\033[0m  %s\n' "$*"; }
fail() { printf '    \033[31mFAIL\033[0m  %s\n' "$*"; failures=$((failures + 1)); }

# Pressing Ctrl-C in a terminal delivers SIGINT to the whole foreground process
# GROUP. `ros2 run` is a wrapper that spawns the node as a child, and it does not
# forward a bare SIGINT, so signalling just its pid would leave the node running
# and make every node look like it ignored Ctrl-C. `setsid` puts the wrapper and
# its child in a new group of their own, so we can signal the group and
# reproduce exactly what a person pressing Ctrl-C does.
# Sets NODE_PID rather than echoing it: a command substitution would run the
# job inside a subshell, and the parent shell cannot `wait` on another shell's
# child -- which silently turns every exit code into 127.
start_node() {   # start_node <executable> <logfile>; sets NODE_PID
    setsid ros2 run robot_control "$1" > "$2" 2>&1 &
    NODE_PID=$!
}

interrupt_node() {   # interrupt_node <pgid>
    kill -INT -"$1" 2>/dev/null || kill -INT "$1" 2>/dev/null
}

wait_for_exit() {   # wait_for_exit <pgid> <deciseconds>; 0 if it exited in time
    local pid="$1" limit="$2" i
    for ((i = 0; i < limit; i++)); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.1
    done
    return 1
}

dump() {  # dump a log, indented, so CI output stays readable
    sed 's/^/          /' "$1" | tail -n 30
}

# Runs on EXIT, including after every check has passed -- so it must never be
# able to block. A bare `wait` here with no timeout is exactly what once made
# this script hang for 20 minutes in CI with all its checks already green.
cleanup() {
    if [ -n "$SIM_PID" ]; then
        # sim.launch.py starts several node processes, so signal the whole group;
        # SIGINT to the launcher alone can leave the children behind.
        kill -INT -"$SIM_PID" 2>/dev/null || kill -INT "$SIM_PID" 2>/dev/null
        if ! wait_for_exit "$SIM_PID" 50; then          # 5s to go quietly
            kill -9 -"$SIM_PID" 2>/dev/null || kill -9 "$SIM_PID" 2>/dev/null
            wait_for_exit "$SIM_PID" 20
        fi
    fi
    rm -rf "$LOGDIR"
}
trap cleanup EXIT

# --------------------------------------------------------------- start the sim
note "Starting the simulator"
# viz_port 0 is not valid, so use the normal one -- nothing else binds it in CI.
# setsid so SIM_PID is also a process-group id, and cleanup can take the whole
# launch tree down rather than just the launcher.
setsid ros2 launch robot_bringup sim.launch.py > "$LOGDIR/sim.log" 2>&1 &
SIM_PID=$!

# Wait for the sim to actually publish, rather than sleeping a fixed guess.
for topic in /scan /odom; do
    if timeout 45 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
        pass "simulator publishes $topic"
    else
        fail "simulator never published $topic after 45s"
        dump "$LOGDIR/sim.log"
        exit 1   # nothing below can pass if the sim is not up
    fi
done

# --------------------------------------------------- every node runs and stops
note "Running every robot_control node for ${RUN_SECONDS}s"
executables="$(ros2 pkg executables robot_control | awk '{print $2}')"
if [ -z "$executables" ]; then
    fail "robot_control has no registered executables at all"
    exit 1
fi

for exe in $executables; do
    log="$LOGDIR/$exe.log"
    start_node "$exe" "$log"; pid="$NODE_PID"

    sleep "$RUN_SECONDS"

    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid"; code=$?
        fail "$exe died after less than ${RUN_SECONDS}s (exit $code)"
        dump "$log"
        continue
    fi

    # Ctrl-C, the way a person would stop it.
    interrupt_node "$pid"

    if ! wait_for_exit "$pid" 100; then       # up to 10s to wind down
        kill -9 -"$pid" 2>/dev/null; kill -9 "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
        fail "$exe ignored Ctrl-C and had to be killed -- on the real robot it would keep driving"
        dump "$log"
        continue
    fi

    wait "$pid"; code=$?
    # 0 = returned from main, 130 = conventional exit-on-SIGINT. Both fine.
    if [ "$code" -eq 0 ] || [ "$code" -eq 130 ]; then
        pass "$exe ran for ${RUN_SECONDS}s and shut down cleanly"
    else
        fail "$exe exited $code on Ctrl-C (expected a clean shutdown)"
        dump "$log"
    fi
done

# ------------------------------------------------ end to end: does it actually move?
if echo "$executables" | grep -qx 'square_driver'; then
    note "End to end: square_driver should move the robot"

    read_x() { timeout 20 ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null | head -1; }

    before="$(read_x)"
    start_node square_driver "$LOGDIR/e2e.log"; pid="$NODE_PID"
    sleep "$RUN_SECONDS"
    after="$(read_x)"
    interrupt_node "$pid"; wait_for_exit "$pid" 100; wait "$pid" 2>/dev/null

    moved="$(python3 -c "
try:
    print('yes' if abs(float('${after:-0}') - float('${before:-0}')) > 0.02 else 'no')
except ValueError:
    print('unknown')
")"
    case "$moved" in
        yes)     pass "robot moved (x: ${before} -> ${after})" ;;
        no)      fail "robot did not move in ${RUN_SECONDS}s (x stayed at ${before}) -- /cmd_vel is not reaching the sim"
                 dump "$LOGDIR/e2e.log" ;;
        *)       fail "could not read pose.pose.position.x from /odom (got '${before}' -> '${after}')" ;;
    esac
fi

# ----------------------------------------------------------------------- verdict
note "Result"
if [ "$failures" -eq 0 ]; then
    echo "    All smoke tests passed."
    exit 0
fi
echo "    $failures check(s) failed."
exit 1
