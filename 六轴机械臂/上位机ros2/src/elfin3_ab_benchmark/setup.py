from glob import glob
from setuptools import find_packages, setup


package_name = "elfin3_ab_benchmark"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="cjw",
    maintainer_email="cjw@todo.todo",
    description="Automated paired MovePose and MovePosePTP benchmark for Elfin3.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "ab_benchmark = elfin3_ab_benchmark.benchmark_node:main",
        ],
    },
)
