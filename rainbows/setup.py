# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from setuptools import setup

package_name = "rainbows"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    package_data={package_name: ["py.typed"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="The AI Institute",
    maintainer_email="engineering@theaiinstitute.com",
    description="The AI Institute's wrappers for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
