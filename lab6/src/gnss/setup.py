import os
from glob import glob
from setuptools import find_packages, setup

package_name = "gnss"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jonathancyriane",
    maintainer_email="li.xinle@northeastern.edu",
    description="Standalone GPS puck driver for NUance Lab 6",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "gnss_node = gnss.gnss_node:main",
        ],
    },
)
