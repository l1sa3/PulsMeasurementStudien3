import os
from glob import glob
from setuptools import setup

package_name = 'pulse_chest_strap'

setup(
    name=package_name,
    version='1.0',
    packages=[package_name],
    #package_dir={'': 'src'},
    maintainer='kck278',
    maintainer_email='kck278@todo.todo',
    description='The pulse_chest_strap package',
    license='TODO',
    install_requires=['setuptools'],
    zip_safe=True,

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.xml')),
    ],

    entry_points={
        'console_scripts': [
            'node = pulse_chest_strap.pulse_chest_strap:main',
        ],
    },
)