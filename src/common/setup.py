#!/usr/bin/env python
from distutils.core import setup

package_name = 'common'

setup(
    name=package_name,
    version='0.0.0',
    packages=['face_detector', 'pulse_publisher', 'bdf_processor'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian Gerkey',
    maintainer_email='gerkey@example.com',
    description='The talker_py package',
    license='BSD',
)
