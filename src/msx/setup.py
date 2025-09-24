from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'msx'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/waldo.sdf']),
        ('share/' + package_name + '/urdf', ['urdf/waldo.urdf']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bastian',
    maintainer_email='bastian.zumbusch@ovgu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msx_pseudo_roofcam_node = msx.msx_pseudo_roofcam_node:main',
            'msx_robot_node = msx.msx_robot_node:main',
            'msx_mock_cam_node = msx.msx_mock_cam_node:main',
        ],
    },
)
