from setuptools import find_packages, setup

package_name = 'tf_rebroadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aimas Lund',
    maintainer_email='s174435@student.dtu.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_lidar_transform = tf_rebroadcaster.imu_lidar_transform:main',
            "imu_rep_transform = tf_rebroadcaster.imu_rep_transform:main",
        ],
    },
)
