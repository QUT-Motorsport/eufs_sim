from os.path import join
from glob import glob
from setuptools import setup, find_packages


package_name = 'eufs_tracks'

share_directory = join('share', package_name)
data_files = [
    (share_directory, ['package.xml', 'plugin.xml']),
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    (join(share_directory, 'csv'), glob('csv/*')),
    (join(share_directory, 'image'), glob('image/*')),
    (join(share_directory, 'launch'),
        glob('launch/*.launch*') + ['launch/blacklist.txt']),
    (join(share_directory, 'materials'), glob('materials/*')),
    (join(share_directory, 'meshes'), glob('meshes/*')),
    (join(share_directory, 'worlds'), glob('worlds/*')),
    (join(share_directory, 'resource/randgen_model_template'),
        glob('resource/randgen_model_template/model.*')),
    (join(share_directory, 'resource'), [
        'resource/noiseFiles.txt',
        'resource/randgen_launch_template',
        'resource/randgen_world_template',
        'resource/conversion_tool.ui',
    ])
]

# Add all model sub-directories
# This is required as setuptools doesn't currently support recursive glob
for directory in glob("models/*"):
    data_file = (join(share_directory, directory), glob(directory + "/*"))
    data_files.append(data_file)

data_files = [
    (share_directory, ['package.xml', 'plugin.xml']),
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    (join(share_directory, 'csv'), glob('csv/*')),
    (join(share_directory, 'image'), glob('image/*')),
    (join(share_directory, 'launch'),
        glob('launch/*.launch*') + ['launch/blacklist.txt']),
    (join(share_directory, 'materials'), glob('materials/*')),
    (join(share_directory, 'meshes'), glob('meshes/*')),
    (join(share_directory, 'worlds'), glob('worlds/*')),
    (join(share_directory, 'resource/randgen_model_template'),
        glob('resource/randgen_model_template/model.*')),
    (join(share_directory, 'resource'), [
        'resource/noiseFiles.txt',
        'resource/randgen_launch_template',
        'resource/randgen_world_template',
        'resource/conversion_tool.ui',
    ])
]

# Add all model sub-directories
# This is required as setuptools doesn't currently support recursive glob
for directory in glob("models/*"):
    data_file = (join(share_directory, directory), glob(directory + "/*"))
    data_files.append(data_file)

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Matthew',
    maintainer_email='cambobmat@icloud.com',
    description='Procedurally generates tracks and converts between file formats.',
    license='MIT',
    tests_require=['pytest'],
    scripts=[
        'scripts/eufs_tracks_converter',
        'scripts/eufs_tracks_generator'
    ],
    entry_points={
        'eufscli.command': [
            'track = eufs_tracks.cli.main:EUFSTrackGenerator'
        ],
        'eufs_tracks.verb': [
            'create = eufs_tracks.cli.create:EUFSTracksCreate',
            'convert = eufs_tracks.cli.convert:EUFSTracksConvert'
        ]
    }
)
