from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar3d_reconstruction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonpo',
    maintainer_email='s194243@student.dtu.dk',
    description='ROS2 package for 3D reconstruction using orthogonal sonar images, inspired by the work of John McConnell and co. (jake3991 on Github).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'acoustic3d_patch = sonar3d_reconstruction.acoustic3d_patch:main',
            'acoustic3d_edge = sonar3d_reconstruction.acoustic3d_edge:main',
        ],
    },
)
