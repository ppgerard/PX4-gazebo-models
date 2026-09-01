#!/usr/bin/env bash
# Build the BodyAero system plugin. Output: build/libBodyAero.so
#
# Put it on Gazebo's search path before starting the simulator:
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=<this dir>/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
# The landing harnesses in px4-smc-control-ws/tools/ do this automatically.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$here/build" -j"$(nproc)"
echo
echo "built: $here/build/libBodyAero.so"
