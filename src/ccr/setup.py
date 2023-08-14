from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ccr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Mai',
    maintainer_email='sebastian.mai@ovgu.de',
    description='implementatdion of the CCR algorithm running distributed on multiple robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ccr_local_planner = ccr.ccr_local_planner:main',
            'ccr_global_planner = ccr.ccr_global_planner:main'
        ],
    },
)
