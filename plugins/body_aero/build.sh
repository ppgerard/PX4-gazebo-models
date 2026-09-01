#!/usr/bin/env bash
# Build the BodyAero system plugin. Output: build/libBodyAero.so
#
# Gazebo finds the library in two independent ways, so every launch route works:
#
#   1. The landing harnesses in px4-smc-control-ws/tools/ prepend this build
#      directory to GZ_SIM_SYSTEM_PLUGIN_PATH themselves.
#   2. This script also symlinks the result into PX4's own gz_plugins build
#      directory, which PX4's generated rootfs/gz_env.sh ALREADY exports on
#      GZ_SIM_SYSTEM_PLUGIN_PATH. That is what makes a bare
#      `make px4_sitl gz_t2_cruza_vtol` work without setting anything.
#
# The symlink lives under build/, so a clean PX4 rebuild removes it; re-run this
# script afterwards. Without it that launch route fails with
#   [Err] [SystemLoader.cc:92] Failed to load system plugin [libBodyAero.so]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$here/build" -j"$(nproc)"

lib="$here/build/libBodyAero.so"
echo
echo "built: $lib"

px4_plugins="$HOME/ros2_workspaces/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins"
if [ -d "$px4_plugins" ]; then
  ln -sfn "$lib" "$px4_plugins/libBodyAero.so"
  echo "linked into PX4's plugin path: $px4_plugins/libBodyAero.so"
else
  echo "NOTE: $px4_plugins does not exist (PX4 not built yet)." >&2
  echo "      Re-run this script after building PX4, or export" >&2
  echo "      GZ_SIM_SYSTEM_PLUGIN_PATH=$here/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >&2
fi
