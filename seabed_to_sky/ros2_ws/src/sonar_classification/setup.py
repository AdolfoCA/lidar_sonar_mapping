from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar_classification'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adolfo',
    maintainer_email='adaca@dtu.dk',
    description='ARGUS semantic classification: Dirichlet-Categorical probabilistic '
                'semantic seabed mapping over fixed, a-priori, config-defined classes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Stage 1-3: measurement-tuple assembly  y_i = (p_i, I_i, l_i, m_i)
            'measurement_node = sonar_classification.measurement_node:main',
            # Stage 4-6: scoring, Dirichlet accumulation, commit
            'classification_node = sonar_classification.classification_node:main',
            # Debug: plot the class-census CSV (one curve per class, one plot)
            'plot_class_census = sonar_classification.plot_class_census:main',
            # Tuning: measure a class's mu/sigma/pi/rho at a known target
            'sample_class = sonar_classification.sample_class:main',
        ],
    },
)
