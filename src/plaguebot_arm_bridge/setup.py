import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'plaguebot_arm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orlando',
    maintainer_email='orlando@plaguebot.dev',
    description='Bridge node entre MoveIt 2 y el firmware ESP32-C6 del brazo PROTON via serial USB',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = plaguebot_arm_bridge.bridge_node:main',
        ],
    },
)
