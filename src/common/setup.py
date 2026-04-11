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
         'bdf_processor.py = common.bdf_processor:main',
         'compare_pulse_values.py = common.compare_pulse_values:main',
         'face_detector.py = common.face_detector:main',
         'pulse_publisher.py = common.pulse_publisher:main',
         'video_input.py = common.video_input:main',
     ],
     },  
)
