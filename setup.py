from glob import glob

from setuptools import find_packages, setup


package_name = "pico_6dof_robot_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial>=3.5"],
    zip_safe=True,
    maintainer="Edgar W",
    maintainer_email="techniccontroller@gmail.com",
    description="ROS 2 serial driver for the Raspberry Pi Pico based 6-DOF robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pico_6dof_driver = pico_6dof_robot_driver.ros2_driver:main",
        ],
    },
)
