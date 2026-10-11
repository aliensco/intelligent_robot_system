#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export METHOD3_WORKSPACE="$task_root/workspace"
export METHOD3_RUNTIME="$task_root/runtime"
export METHOD3_PYTHON=/usr/bin/python2
# Every robot has its own local master. Deliberate overrides remain possible.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
exec bash "$task_root/tools/start_method3_field.sh" "$@"
