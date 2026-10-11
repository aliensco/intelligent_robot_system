#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_arguments=("$@")
set --
source /opt/ros/kinetic/setup.bash
if [ -f "$task_root/workspace/devel/setup.bash" ]; then
  source "$task_root/workspace/devel/setup.bash"
fi
exec /usr/bin/python2 "$task_root/tools/check_method3_environment.py" --root "$task_root" "${task_arguments[@]}"
