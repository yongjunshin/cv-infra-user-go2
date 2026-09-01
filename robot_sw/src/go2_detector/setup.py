from setuptools import find_packages, setup

package_name = "go2_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CV-User team",
    maintainer_email="yjshin@etri.re.kr",
    description="Thin YOLO11n CPU detector for the Go2 patrol app.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "detector_node = go2_detector.detector_node:main",
        ],
    },
)
