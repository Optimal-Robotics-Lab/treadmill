import os
from glob import glob
from setuptools import find_packages, setup

package_name = "treadmill_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob('launch/*.py'),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nathaniel Clark",
    maintainer_email="nec25a@fsu.edu",
    description="GUI for Treadmill control",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "treadmill_gui_node = treadmill_gui.treadmill_gui_node:main"
        ],
    },
)
