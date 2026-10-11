#!/usr/bin/env bash
# Build this copy only; do not start ROS nodes or command the robot.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_skip_system=false
task_jobs=2
while [ "$#" -gt 0 ]; do
  case "$1" in
    --skip-system) task_skip_system=true ;;
    --jobs) task_jobs="$2"; shift ;;
    -h|--help)
      echo "Usage: bash install.sh [--skip-system] [--jobs 2]"
      echo "Install missing dependencies, initialize runtime, and build local workspace. No motion."
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ "$task_jobs" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid --jobs" >&2; exit 2; }
if [ "$(id -u)" -eq 0 ]; then
  echo "Run as your normal Ubuntu user; the installer calls sudo only for system setup." >&2
  exit 2
fi
source /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 16.04 ] || [ ! -f /opt/ros/kinetic/setup.bash ]; then
  echo "This package requires Ubuntu 16.04 with ROS Kinetic already installed." >&2
  exit 2
fi
for task_file in sensors.launch turtlebot_follow_current.launch turtlebot_follow_current.yaml; do
  if [ ! -f "$task_root/profiles/team_current/$task_file" ]; then
    echo "Missing shared profile: profiles/team_current/$task_file. Clone the documented team-handoff branch or use the complete deployment archive." >&2
    exit 2
  fi
done
if [ "$task_skip_system" = false ]; then
  task_missing=()
  while IFS= read -r task_pkg; do
    [[ -z "$task_pkg" || "$task_pkg" = \#* ]] && continue
    if [ "$(dpkg-query -W -f='${Status}' "$task_pkg" 2>/dev/null || true)" != 'install ok installed' ]; then
      task_missing+=("$task_pkg")
    fi
  done < "$task_root/deploy/method3/apt-packages.txt"
  if [ "${#task_missing[@]}" -gt 0 ]; then
    echo "Installing missing packages: ${task_missing[*]}"
    sudo apt-get install -y "${task_missing[@]}"
  fi
  # The distro Kobuki FTDI package supplies its generic /dev/kobuki rule.
  # Camera rules cover Kinect v1 USB endpoints, not a particular serial number.
  sudo install -m 644 "$task_root/deploy/method3/61-method3-kinect.rules" /etc/udev/rules.d/61-method3-kinect.rules
  sudo usermod -a -G dialout,plugdev "$(id -un)"
  sudo udevadm control --reload-rules
  echo "USB rules installed. Newly added group membership needs logout/login; reconnect USB to apply rules."
fi
mkdir -p "$task_root/runtime" "$task_root/workspace/src"
for task_file in sensors.launch turtlebot_follow_current.launch turtlebot_follow_current.yaml; do
  if [ ! -e "$task_root/runtime/$task_file" ]; then
    cp "$task_root/profiles/team_current/$task_file" "$task_root/runtime/$task_file"
  else
    echo "Keeping existing runtime/$task_file"
  fi
done
task_source="$task_root/turtlebot/follow3/rgbd_following"
task_link="$task_root/workspace/src/rgbd_following"
if [ -e "$task_link" ] || [ -L "$task_link" ]; then
  if [ "$(readlink -f "$task_link")" != "$(readlink -f "$task_source")" ]; then
    echo "workspace/src/rgbd_following belongs to another source directory; use a new extraction directory." >&2
    exit 2
  fi
else
  ln -s ../../turtlebot/follow3/rgbd_following "$task_link"
fi
chmod +x "$task_root"/*.sh "$task_root/tools"/*.sh "$task_source/scripts"/*.py
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages
source /opt/ros/kinetic/setup.bash
cd "$task_root/workspace"
catkin_make -j"$task_jobs" -l"$task_jobs" -DCMAKE_PREFIX_PATH=/opt/ros/kinetic -DPYTHON_EXECUTABLE=/usr/bin/python2
bash "$task_root/check.sh"
echo "Installed. Preview: bash run.sh --view-only | Follow + avoidance: bash run.sh"
