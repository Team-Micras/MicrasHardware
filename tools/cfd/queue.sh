#!/bin/bash
# Run a list of jobs one after another, detached (survives the shell), logging to build/cfd/runs/queue.log:
#   tools/cfd/queue.sh "<case> mesh" "<case> solve 1000 1500" "<case> solve 800 800" ... "walk <case> 600 900 950 1000"
# Stops at the first failure. One queue at a time (memory): it refuses to start if another is running.
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
LOG=$ROOT/build/cfd/runs/queue.log
LOCK=$ROOT/build/cfd/runs/queue.lock
if [ -e "$LOCK" ] && kill -0 "$(cat "$LOCK")" 2>/dev/null; then echo "queue already running (pid $(cat "$LOCK"))"; exit 1; fi
setsid nohup bash -c '
    echo $$ > "'"$LOCK"'"
    for job in "$@"; do
        echo "[$(date "+%F %T")] START $job"
        case "$job" in walk\ *) cmd="'"$ROOT"'/tools/cfd/walk.sh ${job#walk }";; *) cmd="'"$ROOT"'/tools/cfd/cfd.sh $job";; esac
        if ! $cmd; then echo "[$(date "+%F %T")] FAILED $job"; rm -f "'"$LOCK"'"; exit 1; fi
        echo "[$(date "+%F %T")] DONE $job"
    done
    echo "[$(date "+%F %T")] QUEUE EMPTY"
    rm -f "'"$LOCK"'"
' queue "$@" >> "$LOG" 2>&1 < /dev/null &
echo "queued ${#@} jobs; log: $LOG"
