from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = "eufs_racecar"
share_directory = join("share", package_name)

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share_directory, ["package.xml"]),
        (join(share_directory, "meshes"), glob("meshes/*.dae")),
        (join(share_directory, "urdf"), glob("urdf/*.urdf.xacro")),
        (share_directory, ["racecars.txt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cameron Matthew",
    maintainer_email="cambobmat@icloud.com",
    description="Launch, URDF, mesh and material files for autonomous vehicles.",
    license="MIT",
    tests_require=["pytest"],
)
