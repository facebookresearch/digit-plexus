#!/usr/bin/env python
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import setuptools
from setuptools import find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="digitplexus",
    version="0.0.1",
    packages=find_packages(include=['plexus/ros2/plexus_pkg']),
    author="Meta Research",
    description="<fill>",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/facebookresearch/digit-plexus",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    python_requires=">=3.8",
    install_requires=[
       "pyserial==3.5",
        "cobs==1.2.0",
    ],
)