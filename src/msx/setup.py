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
            'sunburst_skyview_calc = msx.sunburst_skyview_calc:main',
            'sunburst_robot_calc = msx.sunburst_robot_calc:main',
            'skyview_finder = msx.skyview_finder:main',
            'gt_formatting = msx.gt_formatting:main',
        ],
    },
)
