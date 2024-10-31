
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/bin/bash -e
if [ "x$ROS_DISTRO" = "x" ]; then
	echo "please first initialize ROS env (e.g., conda activate _plexus)"
	exit
fi

if git rev-parse --git-dir >/dev/null 2>&1; then
	REPOROOT=$(git rev-parse --show-toplevel)
else
	REPOROOT=$PWD/src/digit-plexus/
fi

if [ $(basename $PWD) = "plexus_ws" ]; then
	colcon build --base-paths $REPOROOT/plexus/ros2 \
		--packages-select plexus_pkg \
		--cmake-args "-DPython3_EXECUTABLE=$(which python3)" --cmake-clean-cache \
		--symlink-install --event-handlers console_direct+
else
	echo please invoke from plexus_ws
fi

echo "please"
echo
echo "source ./install/setup.$(basename $SHELL)"
