from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'plaguebot_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'),
            glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jada',
    maintainer_email='orlandocamc@gmail.com',
    description='Vision package for plaguebot using Orbbec depth camera with WebRTC streaming bridge for Meta Quest 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webrtc_bridge = plaguebot_vision.webrtc_bridge:main',
        ],
    },
)
