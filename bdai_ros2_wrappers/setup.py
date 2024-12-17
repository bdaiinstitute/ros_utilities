# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from setuptools import find_packages, setup

package_name = "bdai_ros2_wrappers"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["py.typed"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="The AI Institute",
    maintainer_email="opensource@theaiinstitute.com",
    zip_safe=True,
    description="The AI Institute's wrappers for ROS 2",
    license="MIT",
)
