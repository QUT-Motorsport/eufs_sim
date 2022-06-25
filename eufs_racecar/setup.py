from os.path import join
from glob import glob
from setuptools import setup, find_packages

package_name = 'eufs_racecar'

share_directory = join('share', package_name)

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (share_directory, ['package.xml']),
    (join(share_directory, 'meshes'), glob('meshes/*.dae')),
    (join(share_directory, 'urdf'), glob('urdf/*.urdf.xacro')),
    (join(share_directory, 'launch'), glob('launch/*.launch.py')),

]

# Add all robot sub-directories
# This is required as setuptools doesn't currently support recursive glob
for directory in glob("robots/*"):
    data_file = (join(share_directory, directory), glob(directory + "/*"))
    data_files.append(data_file)


setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Matthew',
    maintainer_email='cambobmat@icloud.com',
    description='Launch, URDF, mesh and material files for autonomous vehicles.',
    license='MIT',
    tests_require=['pytest'],
)
