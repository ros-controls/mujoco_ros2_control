#!/usr/bin/env python3

from setuptools import setup

setup(
    name='mujoco_ros2_control',
    version='0.0.0',
    packages=['mujoco_ros2_control', 'mujoco_ros2_control.tests'],
    tests_require=['pytest'],
)
