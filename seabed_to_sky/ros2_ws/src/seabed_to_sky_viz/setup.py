import os
from glob import glob

from setuptools import setup

package_name = 'seabed_to_sky_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='adocafaro@gmail.com',
    description='Publish saved mission lidar + sonar clouds (separate / per-sensor / fused) for Foxglove.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seabed_to_sky_viz = seabed_to_sky_viz.map_publisher_node:main',
        ],
    },
)
