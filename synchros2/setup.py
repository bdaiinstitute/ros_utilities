# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

from setuptools import find_packages, setup

package_name = "synchros2"

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
    maintainer="RAI Institute",
    maintainer_email="opensource@rai-inst.com",
    description="RAI Institute wrappers for ROS2",
    tests_require=["pytest"],
    zip_safe=True,
    license="MIT",
)
