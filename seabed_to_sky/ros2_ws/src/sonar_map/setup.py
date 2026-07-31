from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar_map'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer_email='adaca@dtu.dk',
    description='Sonar mapping package: transforms sonar scans using odometry and accumulates into a probabilistic 3D voxel map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_scan_ned = sonar_map.sonar_scan_ned:main',
            'sonar_map_ned_no_semantic = sonar_map.sonar_map_ned_no_semantic:main',
            'save_map = sonar_map.save_map:main',
            # Dirichlet-Categorical voxel mapping pipeline
            'seabed_estimator = sonar_map.seabed_estimator:main',
            'return_classifier = sonar_map.return_classifier:main',
            'voxel_mapper = sonar_map.voxel_mapper:main',
            'learned_voxel_mapper = sonar_map.learned_voxel_mapper:main',
            'seabed_surface = sonar_map.seabed_surface:main',
            'lidar_count_debug = sonar_map.lidar_count_debug:main',
            'lidar_support_csv_logger = sonar_map.lidar_support_csv_logger:main',
        ],
    },
)
