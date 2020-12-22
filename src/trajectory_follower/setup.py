from setuptools import setup

package_name = 'trajectory_follower'

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
    maintainer='basti',
    maintainer_email='sebastian.mai@ovgu.de',
    description='a packge with controllers to make a robot follow a trajectory',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_follower = trajectory_follower.trajectory_follower:main'
        ],
    },
)
