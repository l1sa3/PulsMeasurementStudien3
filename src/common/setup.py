#!/usr/bin/env python
import os
from glob import glob
from setuptools import setup

package_name = 'common'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #packages=['face_detector', 'pulse_publisher', 'bdf_processor'],
    #package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver',
    maintainer_email='oliver@todo.todo',
    description='The common package',
    license='TODO',

    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],

    entry_points={
     'console_scripts': [
         'node1 = common.bdf_processor:main',
         'node2 = common.compare_pulse_values:main',
         'node3 = common.face_detector:main',
         'node4 = common.pulse_publisher:main',
         'node5 = common.video_input:main',
     ],
     },  
)
