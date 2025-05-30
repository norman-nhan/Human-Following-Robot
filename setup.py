from setuptools import find_packages, setup
import os
from glob import glob

package_name = "lecture04_pkg"

# サブモジュールリストを定義(自作モジュール)
submodules = [
    f"{package_name}/state_sample1",
    f"{package_name}/state_sample2",
    f"{package_name}/state_main",
    f"{package_name}/state_exercise",
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", *submodules]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("./launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tomoaki Fujino",
    maintainer_email="fujino0728@gmail.com",
    description="Lecture04: State Machine(YASMIN), roslaunch2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sm_sample1 = lecture04_pkg.sm_sample1:main",
            "sm_sample2 = lecture04_pkg.sm_sample2:main",
            "sm_exercise = lecture04_pkg.sm_exercise:main",
            "sm_main = lecture04_pkg.sm_main:main",
        ],
    },
)
