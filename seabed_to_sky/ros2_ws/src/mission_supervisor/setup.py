from glob import glob
import os

from setuptools import setup

package_name = 'mission_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='adocafaro@gmail.com',
    description='Long-mission mapping health controller (latency-triggered save/prune).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_supervisor = mission_supervisor.mission_supervisor_node:main',
            'fuse_maps = mission_supervisor.fuse_maps:main',
            'fuse_lidar_icp = mission_supervisor.fuse_lidar_icp:main',
            'publish_pcd = mission_supervisor.publish_pcd:main',
            'publish_lidar_chunks = mission_supervisor.publish_lidar_chunks:main',
        ],
    },
)
