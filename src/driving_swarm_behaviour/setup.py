from setuptools import setup

package_name = 'driving_swarm_behaviour'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Mai',
    maintainer_email='sebastian.mai@ovgu.de',
    description='simple behaviours for robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive = driving_swarm_behaviour.reactive:main'
        ],
    },
)
