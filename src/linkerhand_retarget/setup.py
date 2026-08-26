#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
from setuptools import find_packages, setup

package_name = 'linkerhand_retarget'

this_dir = os.path.abspath(os.path.dirname(__file__))
custom_dir = os.path.join(this_dir, package_name, "LinkerHand")

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='2.12.10',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=[
        'setuptools',
        'numpy',
        'scipy>=1.10.0',
        'PyYAML',
        'transforms3d',
        'anytree',
        'loguru',
        'trimesh',
        'tqdm',
        'colorama',
        'lxml',
        'pyserial',
        'six',
        'tyro',
    ],
    zip_safe=True,
    maintainer='Linker Robotics',
    maintainer_email='support@linker-robotics.com',
    description='ROS2 SDK for Linker Hand Teleoperation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'handretarget = linkerhand_retarget.handretarget:main',
        ],
    },
)
