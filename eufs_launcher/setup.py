from glob import glob
from os.path import join

from setuptools import setup

package_name = "eufs_launcher"
share_directory = join("share", package_name)

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        (share_directory, glob("launch/*.launch.py")),
        (join(share_directory, "ui"), ["ui/launcher.ui"]),
        (share_directory, ["plugin.xml"]),
        (share_directory, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    entry_points={
        'console_scripts': [
            'cone_detection_converter = eufs_launcher.cone_detection_converter:main'
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cameron Matthew",
    maintainer_email="cambobmat@icloud.com",
    description="Configures and launches eufs_sim.",
    license="MIT",
    tests_require=["pytest"],
    scripts=["scripts/eufs_launcher"],
)
