from setuptools import find_packages, setup

package_name = "rpi_gpiozero_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/gpiozero_node.launch.py"]),
        (f"share/{package_name}/config", ["config/gpiozero_params.yaml"]),
    ],
    install_requires=["setuptools", "gpiozero"],
    zip_safe=True,
    maintainer="anthony",
    maintainer_email="anthony@example.com",
    description="ROS 2 Jazzy package for Raspberry Pi 5 GPIO with gpiozero",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gpiozero_node = rpi_gpiozero_ros.gpio_node:main",
        ],
    },
)
