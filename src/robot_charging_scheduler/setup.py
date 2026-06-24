from setuptools import find_packages, setup

package_name = "robot_charging_scheduler"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="ROS2 Python simulation for robot charging scheduling.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "charging_scheduler = robot_charging_scheduler.charging_scheduler_node:main",
        ],
    },
)
