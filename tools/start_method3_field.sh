#!/usr/bin/env bash
set -eo pipefail
task_arguments=("$@")
set --
source /home/haiden/codex_method3_ws/devel/setup.bash
set -- "${task_arguments[@]}"
if ! timeout 3 rosnode list >/dev/null 2>&1; then
  mkdir -p "$HOME/codex_method3_runtime"
  nohup roscore > "$HOME/codex_method3_runtime/roscore.log" 2>&1 < /dev/null &
  for task_attempt in {1..20}; do
    if timeout 1 rosnode list >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
task_tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/codex_method3_runtime"
# Close the lock in descendants: the camera and RViz deliberately outlive the
# motion session, and must not keep a completed session locked forever.
if flock --nonblock --close --conflict-exit-code 75 \
    "$HOME/codex_method3_runtime/field_session.lock" \
    python -u "$task_tools_dir/run_rgbd_field_session.py" --start-stack --allow-unobserved "$@"; then
  exit 0
else
  task_status=$?
  if [ "$task_status" -eq 75 ]; then
    echo "A method3 field session is already running. Use its terminal; Ctrl+C stops it." >&2
  fi
  exit "$task_status"
fi
