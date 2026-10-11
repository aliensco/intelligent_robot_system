#!/usr/bin/env bash
set -eo pipefail
task_arguments=("$@")
task_tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_bundle_root="$(cd -- "$task_tools_dir/.." && pwd)"
task_python="${METHOD3_PYTHON:-python}"
task_workspace="${METHOD3_WORKSPACE:-$task_bundle_root/workspace}"
task_runtime="${METHOD3_RUNTIME:-$task_bundle_root/runtime}"
# Keep the current operator's old entry point usable while new bundles use
# only paths relative to their own installation, independent of username.
if [ -z "${METHOD3_WORKSPACE:-}" ] &&
   [ ! -f "$task_workspace/devel/setup.bash" ] &&
   [ -f "$HOME/codex_method3_ws/devel/setup.bash" ]; then
  task_workspace="$HOME/codex_method3_ws"
  task_runtime="${METHOD3_RUNTIME:-$HOME/codex_method3_runtime}"
fi
# Resolve the runtime before taking its lock: CLI selection, logs and the
# Python supervisor must all refer to the same directory.
while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) exec "$task_python" "$task_tools_dir/run_rgbd_field_session.py" --help ;;
    --runtime)
      if [ "$#" -lt 2 ] || [ -z "$2" ]; then
        echo "--runtime requires a directory." >&2
        exit 2
      fi
      task_runtime="$2"
      shift
      ;;
    --runtime=*) task_runtime="${1#--runtime=}" ;;
  esac
  shift
done
case "$task_workspace" in
  '~') task_workspace="$HOME" ;;
  '~/'*) task_workspace="$HOME/${task_workspace#\~/}" ;;
esac
case "$task_runtime" in
  '~') task_runtime="$HOME" ;;
  '~/'*) task_runtime="$HOME/${task_runtime#\~/}" ;;
esac
if [ -z "$task_runtime" ]; then
  echo "--runtime requires a directory." >&2
  exit 2
fi
if [ ! -f "$task_workspace/devel/setup.bash" ]; then
  echo "Method3 workspace is not built: $task_workspace/devel/setup.bash" >&2
  echo "Run the bundle installer or set METHOD3_WORKSPACE to your built catkin workspace." >&2
  exit 2
fi
task_workspace="$(cd -- "$task_workspace" && pwd)"
mkdir -p -- "$task_runtime"
task_runtime="$(cd -- "$task_runtime" && pwd)"
export METHOD3_WORKSPACE="$task_workspace" METHOD3_RUNTIME="$task_runtime"
set --
source "$task_workspace/devel/setup.bash"
set -- "${task_arguments[@]}"
if ! timeout 3 rosnode list >/dev/null 2>&1; then
  nohup roscore > "$task_runtime/roscore.log" 2>&1 < /dev/null &
  for task_attempt in {1..20}; do
    if timeout 1 rosnode list >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
# Close the lock in descendants: the camera and RViz deliberately outlive the
# motion session, and must not keep a completed session locked forever.
if flock --nonblock --close --conflict-exit-code 75 \
    "$task_runtime/field_session.lock" \
    "$task_python" -u "$task_tools_dir/run_rgbd_field_session.py" --start-stack --allow-unobserved "$@"; then
  exit 0
else
  task_status=$?
  if [ "$task_status" -eq 75 ]; then
    echo "A method3 field session is already running. Use its terminal; Ctrl+C stops it." >&2
  fi
  exit "$task_status"
fi
