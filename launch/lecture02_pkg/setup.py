from setuptools import find_packages, setup

package_name = "lecture02_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tomoaki Fujino",
    maintainer_email="fujino0728@gmail.com",
    description="Lecture02: rosbag2, rviz2, rqt, CvBridge, tf2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "opencv_sample = lecture02_pkg.opencv_sample:main",
            "opencv_exercise  = lecture02_pkg.opencv_exercise:main",
            "tf2_broadcaster = lecture02_pkg.tf2_broadcaster:main",
            "tf2_listener  = lecture02_pkg.tf2_listener:main",
        ],
    },
)
