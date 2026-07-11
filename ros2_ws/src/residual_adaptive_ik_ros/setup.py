from setuptools import find_packages, setup

package_name = "residual_adaptive_ik_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "residual_adaptive_ik_ros/launch/residual_ik.launch.py",
            "residual_adaptive_ik_ros/launch/hardware_test.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spark_isaac_mycobot_v2",
    maintainer_email="dev@example.com",
    description="ROS 2 residual adaptive IK for MyCobot 280",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "residual_ik_node = residual_adaptive_ik_ros.residual_ik_node:main",
            "hardware_test_node = residual_adaptive_ik_ros.hardware_test_node:main",
        ],
    },
)
