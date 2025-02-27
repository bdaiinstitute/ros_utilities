# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

from setuptools import setup

package_name = "bdai_ros2_wrappers"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="The AI Institute",
    maintainer_email="opensource@theaiinstitute.com",
    description="The AI Institute's wrappers for ROS2",
    tests_require=["pytest"],
    zip_safe=True,
    license="MIT",
)
