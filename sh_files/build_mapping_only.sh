#!/usr/bin/env bash
# Prepare ROS1 driver metadata without the upstream build.sh workspace deletion.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
source /opt/ros/noetic/setup.bash
cd "$WORKSPACE"
driver="$WORKSPACE/src/livox_ros_driver2"
if [[ ! -f "$driver/package.xml" ]]; then
    cp "$driver/package_ROS1.xml" "$driver/package.xml"
elif ! cmp -s "$driver/package_ROS1.xml" "$driver/package.xml"; then
    echo 'Existing Livox manifest differs from ROS1 template; check it before building.' >&2
    exit 1
fi
cmake -S "$WORKSPACE/src" -B "$WORKSPACE/build" \
    -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_DEVEL_PREFIX="$WORKSPACE/devel" -DCMAKE_INSTALL_PREFIX="$WORKSPACE/install"
cmake --build "$WORKSPACE/build" --target run_mapping_online livox_ros_driver2_node -- -j2 -l2
