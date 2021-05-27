from setuptools import setup

package_name = 'eufs_tracks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['launch/eufs_track_generator.launch.py']),
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
