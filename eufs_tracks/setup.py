from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = "eufs_tracks"

share_directory = join("share", package_name)
data_files = [
    (share_directory, ["package.xml", "plugin.xml"]),
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    (join(share_directory, "csv"), glob("csv/*")),
    (join(share_directory, "image"), glob("image/*")),
    (join(share_directory, "meshes"), glob("meshes/*")),
    (join(share_directory, "ui"), glob("ui/*")),
    (join(share_directory, "worlds"), glob("worlds/*")),
]

# Add all model sub-directories
# This is required as setuptools doesn't currently support recursive glob
for directory in glob("models/*"):
    data_file = (join(share_directory, directory), glob(directory + "/*"))
    data_files.append(data_file)

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    author="Cameron Matthew",
    author_email="cambobmat@icloud.com",
    maintainer="Alastair Bradford",
    maintainer_email="albradford2468@gmail.com",
    description="Procedurally generates tracks and converts between file formats.",
    license="MIT",
    tests_require=["pytest"],
    scripts=["scripts/eufs_tracks_converter", "scripts/eufs_tracks_generator"],
)
