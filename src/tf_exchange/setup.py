import os
from glob import glob
from setuptools import setup

package_name = 'tf_exchange'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='traichel',
    maintainer_email='nele.traichel@ovgu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_tf_pub = tf_exchange.local_tf_pub:main',
            'local_to_global_tf_pub = tf_exchange.local_to_global_tf_pub:main',
            'global_to_local_tf_pub= tf_exchange.global_to_local_tf_pub:main'
        ],
    },
)
