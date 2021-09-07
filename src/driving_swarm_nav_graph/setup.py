from setuptools import setup
import os
from glob import glob

package_name = 'driving_swarm_nav_graph'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Mai',
    maintainer_email='sebastian.mai@ovgu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_graph = driving_swarm_nav_graph.nav_graph:main',
            'nav_graph_planner = driving_swarm_nav_graph.nav_graph_planner:main',
            'global_graph_planner = driving_swarm_nav_graph.global_graph_planner:main',
        ],
    },
)
