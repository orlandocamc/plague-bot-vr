from setuptools import find_packages, setup

package_name = 'plaguebot_py_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orlando',
    maintainer_email='orlando@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_parameter = plaguebot_py_examples.simple_parameter:main',
            'simple_service_server = plaguebot_py_examples.simple_service_server:main',
            'simple_service_client = plaguebot_py_examples.simple_service_client:main'
        ],
    },
)
