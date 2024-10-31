# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/bin/bash -e
shopt -s expand_aliases
if type -P micromamba; then
	echo "micromamba detect, using micromamba inplace of mamba"
	alias conda=micromamba
	eval "$(micromamba shell hook --shell bash)"
elif type -P mamba || type -P conda; then
	eval "$(conda shell.bash hook)"
else
	echo please install mamba or micromamba
	exit
fi


# if ENV_NAME is not set, then set it to _plexus

ENV_NAME=_plexus

unset PYTHONPATH LD_LIBRARY_PATH

# remove any exisiting env
mamba remove -y -n $ENV_NAME --all || true #will fail in fresh install

# create new env for ROS
mamba create -y --name $ENV_NAME --override-channels --no-channel-priority -c tingfan -c conda-forge python=3.8 \
  python-can compilers cmake pkg-config make ninja colcon-common-extensions \
  ros-humble-ros-base ros-humble-foxglove-bridge ros-humble-realsense2-camera ros-humble-ros1-bridge ros-humble-allegro-hand-controllers \
  ros-humble-rosbag2-storage-mcap ros-humble-compressed-image-transport ros-humble-allegro-hand-controllers ros-humble-xacro ros-humble-control-msgs \
  ros-humble-joint-state-publisher libopencv=*=py* \
  portaudio 

conda activate $ENV_NAME
pip install termcolor tensorboardX gitpython pytest==8.0.1 pytest-md-report setuptools==58.2.0 pre-commit
pip install sophuspy scipy matplotlib
pip install cython omegaconf hydra-core digit-interface

# Remove pyudev directory that causes OSError
rm -fr $CONDA_PREFIX/lib/udev
