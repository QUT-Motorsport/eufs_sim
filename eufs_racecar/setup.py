from os.path import join
from glob import glob
from setuptools import setup, find_packages

package_name = 'eufs_racecar'

share_directory = join('share', package_name)

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (share_directory, ['package.xml']),
        (join(share_directory, 'launch'), glob('launch/*.launch*')),
        (join(share_directory, 'materials'), glob('materials/*')),
        (join(share_directory, 'meshes'), glob('meshes/*.dae')),
        (join(share_directory, 'urdf'), glob('urdf/*.urdf.xacro')),
        (join(share_directory, 'robots/ads-dv'),
         glob('robots/ads-dv/*.yaml') + glob('robots/ads-dv/*.urdf.xacro')),
        (join(share_directory, 'robots/ads-dv-2021'),
         glob('robots/ads-dv-2021/*.yaml') + glob(
             'robots/ads-dv-2021/*.urdf.xacro')),
        (join(share_directory, 'robots/eufs'),
         glob('robots/eufs/*.yaml') + glob('robots/eufs/*.urdf.xacro')),
        (join(share_directory, 'robots/wheelchair-21'),
         glob('robots/wheelchair-21/*.yaml') + glob(
             'robots/wheelchair-21/*.urdf.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Matthew',
    maintainer_email='cambobmat@icloud.com',
    description='Launch, URDF, mesh and material files for autonomous vehicles.',
    license='MIT',
    tests_require=['pytest'],
)
