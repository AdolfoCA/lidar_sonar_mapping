from setuptools import find_packages, setup

package_name = 'demo_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adolfo',
    maintainer_email='adaca@dtu.dk',
    description='Demo orchestrator — start/stop the full seabed-to-sky pipeline from Foxglove.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'demo_manager = demo_manager.demo_manager_node:main',
        ],
    },
)
