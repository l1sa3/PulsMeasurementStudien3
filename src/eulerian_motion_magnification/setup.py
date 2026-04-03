
import os
from glob import glob
from setuptools import setup

package_name = 'eulerian_motion_magnification'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={'': 'src'},
    maintainer='sophia',
    maintainer_email='sophia@todo.todo',
    description='The eulerian_motion_magnification package',
    license='TODO',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
    ],

    entry_points={
        'console_scripts': [
            'node = eulerian_motion_magnification.eulerian_motion_magnification:main',
        ],
    },
)