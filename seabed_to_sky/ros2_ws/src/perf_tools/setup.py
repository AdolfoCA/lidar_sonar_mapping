from setuptools import setup
import os
from glob import glob

package_name = 'perf_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'config.yaml']),
        (os.path.join('share', package_name), glob('run_perf_capture.sh')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='adocafaro@gmail.com',
    description='Performance-measurement suite for the seabed-to-sky mapping stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Each script is ALSO standalone-runnable as `python3 <file> --help`;
            # these entry points just make them available via `ros2 run`.
            'latency_sniffer = perf_tools.latency_sniffer:main',
            'resource_logger = perf_tools.resource_logger:main',
            'map_growth_logger = perf_tools.map_growth_logger:main',
            'plot_perf_mission = perf_tools.plot_perf_mission:main',
        ],
    },
)
