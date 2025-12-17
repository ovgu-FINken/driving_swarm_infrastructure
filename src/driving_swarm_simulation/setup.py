from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'driving_swarm_simulation'

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
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Mai',
    maintainer_email='sebastian.mai@ovgu.de',
    description='Launch and world files for gazebo simulation with tb3',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
