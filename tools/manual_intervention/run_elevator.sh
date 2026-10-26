#!/usr/bin/env bash
# Start inside the simulation container with the shared ROS/DDS environment loaded.
set -e
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
command -v ros2 >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
export TINYNAV_WORLD_NAME=elevator TINYNAV_GO2_MOTION=kinematic
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/gazebo/scene/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export __GLX_VENDOR_LIBRARY_NAME=nvidia __NV_PRIME_RENDER_OFFLOAD=1
if [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
log_dir=${TINYNAV_ELEVATOR_LOG_DIR:-/tmp/tinynav-elevator}
mkdir -p "$log_dir"
# Some runtime images mount the NVIDIA EGL library but omit its GLVND manifest.
if [[ ! -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]] && ldconfig -p | grep -q libEGL_nvidia; then
  printf '%s\n' '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > "$log_dir/nvidia-egl.json"
  export __EGL_VENDOR_LIBRARY_FILENAMES="$log_dir/nvidia-egl.json"
fi
trap 'trap - TERM INT EXIT; kill $(jobs -pr) 2>/dev/null || true; wait' TERM INT EXIT
ign gazebo -s -r --headless-rendering -v 3 "$ROOT/gazebo/worlds/elevator.sdf" > "$log_dir/gazebo.log" 2>&1 &
bash "$ROOT/gazebo/robots/go2/spawn.sh" > "$log_dir/spawn.log" 2>&1 &
python3 "$ROOT/tools/manual_intervention/elevator_node.py" > "$log_dir/elevator.log" 2>&1 &
wait -n
