from setuptools import setup
from glob import glob

package_name = "ros2_workshop"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf.xacro")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
        ("share/" + package_name + "/map", ["map/map.pgm"]),
        ("share/" + package_name + "/map", ["map/map.yaml"]),
        ("share/" + package_name + "/sdf", ["sdf/visualize_lidar.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Danny Ng",
    maintainer_email="ngwk@utar.edu.my",
    description="Ros 2 workshop examples",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
