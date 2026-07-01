from glob import glob

from setuptools import find_packages, setup

package_name = "plaguebot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/meshes/visual", glob("meshes/visual/*")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Orlando",
    maintainer_email="orlandocamc@gmail.com",
    description="Real-robot bringup for Plague-Bot VR (ESP32 bridge, EKF, SLAM, Nav2).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "esp32_bridge = plaguebot_bringup.esp32_bridge:main",
            "detection_adapter = plaguebot_bringup.detection_adapter:main",
            "serial_sniffer = plaguebot_bringup.serial_sniffer:main",
        ],
    },
)
