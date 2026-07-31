from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'otter_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adolfo',
    maintainer_email='todo@todo.com',
    description='Navigation package for the Otter USV: converts odometry to NMEA sentences over TCP.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_to_nmea = otter_navigation.odometry_to_nmea:main',
        ],
    },
)
