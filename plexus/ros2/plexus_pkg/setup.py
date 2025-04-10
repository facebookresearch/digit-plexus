
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


from setuptools import find_packages, setup

package_name = 'plexus_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['plexus_pkg']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/allegro_hand_launch.py', 'launch/sensor_patch_launch.py', 'launch/tmr_hand_launch.py',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='chaitanyantr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        
    },
)