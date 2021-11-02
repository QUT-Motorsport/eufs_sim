from os.path import join
from glob import glob
from setuptools import setup

package_name = 'eufs_tracks'

share_directory = join('share', package_name)

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        (share_directory, ['package.xml', 'plugin.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (join(share_directory, 'csv'), glob('csv/*')),
        (join(share_directory, 'image'), glob('image/*')),
        (join(share_directory, 'launch'),
         glob('launch/*.launch*') + ['launch/blacklist.txt']),
        (join(share_directory, 'materials'), glob('materials/*')),
        (join(share_directory, 'meshes'), glob('meshes/*')),
        (join(share_directory, 'models/acceleration'),
         glob('models/acceleration/model.*')),
        (join(share_directory, 'models/big_cone'),
         glob('models/big_cone/model.*')),
        (join(share_directory, 'models/blue_cone'),
         glob('models/blue_cone/model.*')),
        (join(share_directory, 'models/ground_plane'),
         glob('models/ground_plane/model.*')),
        (join(share_directory, 'models/orange_cone'),
         glob('models/orange_cone/model.*')),
        (join(share_directory, 'models/rand'), glob('models/rand/model.*')),
        (join(share_directory, 'models/skidpad'),
         glob('models/skidpad/model.*')),
        (join(share_directory, 'models/small_track'),
         glob('models/small_track/model.*')),
        (join(share_directory, 'models/yellow_cone'),
         glob('models/yellow_cone/model.*')),
        (join(share_directory, 'worlds'), glob('worlds/*')),
        (join(share_directory, 'resource/randgen_model_template'),
         glob('resource/randgen_model_template/model.*')),
        (join(share_directory, 'resource'), [
            'resource/noiseFiles.txt',
            'resource/randgen_launch_template',
            'resource/randgen_world_template',
            'resource/track_generator.ui',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Matthew',
    maintainer_email='cambobmat@icloud.com',
    description='Procedurally generates tracks and converts between file formats.',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/eufs_tracks'],
)
