from setuptools import setup
import os
from glob import glob

package_name = 'driving_swarm_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
    description='launch files for all things on the real turtlebot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = driving_swarm_core.demo:main'
        ],
    },
)
