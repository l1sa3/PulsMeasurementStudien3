#!/usr/bin/env python
from setuptools import setup

package_name = 'common'

setup(
    name=package_name,
    version='0.0.0',
    packages=['face_detector', 'pulse_publisher', 'bdf_processor'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver',
    maintainer_email='oliver@todo.todo',
    description='The common package',
    license='TODO',

    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ],

    # entry_points={
    # 'console_scripts': [
    #     'talker_py_node = talker_py:main',
    # ],
    # },  
)
