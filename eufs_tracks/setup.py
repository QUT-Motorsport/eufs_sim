from setuptools import setup

package_name = 'eufs_tracks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name + '/csv', [
            'csv/acceleration.csv',
            'csv/rand.csv',
            'csv/skidpad.csv',
            'csv/small_track.csv'
        ]),
        ('share/' + package_name + '/image', [
            'image/skidpad.png'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/eufs_track_generator.launch.py',
            'launch/acceleration.launch',
            'launch/empty.launch',
            'launch/rand.launch',
            'launch/skidpad.launch',
            'launch/small_track.launch',
            'launch/blacklist.txt'
        ]),

        ('share/' + package_name + '/materials', [
            'materials/asphalt.jpg'
        ]),
        ('share/' + package_name + '/meshes', [
            'meshes/cone.dae',
            'meshes/cone_big.dae',
            'meshes/cone_blue.dae',
            'meshes/cone_yellow.dae'
        ]),
        ('share/' + package_name + '/models/acceleration', [
            'models/acceleration/model.config',
            'models/acceleration/model.sdf'
        ]),
        ('share/' + package_name + '/models/big_cone', [
            'models/big_cone/model.config',
            'models/big_cone/model.sdf'
        ]),
        ('share/' + package_name + '/models/blue_cone', [
            'models/blue_cone/model.config',
            'models/blue_cone/model.sdf'
        ]),
        ('share/' + package_name + '/models/ground_plane', [
            'models/ground_plane/model.config',
            'models/ground_plane/model.sdf'
        ]),
        ('share/' + package_name + '/models/orange_cone', [
            'models/orange_cone/model.config',
            'models/orange_cone/model.sdf'
        ]),
        ('share/' + package_name + '/models/rand', [
            'models/rand/model.config',
            'models/rand/model.sdf'
        ]),
        ('share/' + package_name + '/models/skidpad', [
            'models/skidpad/model.config',
            'models/skidpad/model.sdf'
        ]),
        ('share/' + package_name + '/models/small_track', [
            'models/small_track/model.config',
            'models/small_track/model.sdf'
        ]),
        ('share/' + package_name + '/models/yellow_cone', [
            'models/yellow_cone/model.config',
            'models/yellow_cone/model.sdf'
        ]),
        ('share/' + package_name + '/resource', [
            'resource/track_generator.ui',
            'resource/randgen_launch_template',
            'resource/randgen_world_template',
            'resource/noiseFiles.txt'
        ]),
        ('share/' + package_name + '/resource/randgen_model_template', [
            'resource/randgen_model_template/model.config',
            'resource/randgen_model_template/model.sdf'
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/acceleration.world',
            'worlds/LAST_LAUNCH.world',
            'worlds/rand.world',
            'worlds/skidpad.world',
            'worlds/small_track.world'
        ]),
        ('share/' + package_name, ['plugin.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'scipy', 'pandas'],
    zip_safe=True,
    maintainer='Angus Stewart',
    maintainer_email='siliconlad@protonmail.com',
    description='This package relates to the generation and formatting of tracks to be launched in our simulation.',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/eufs_tracks'],
)
