#!/usr/bin/env bash
set -eo pipefail
source /home/haiden/codex_method3_ws/devel/setup.bash
task_tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python -u "$task_tools_dir/run_rgbd_field_session.py" --start-stack --allow-unobserved "$@"
