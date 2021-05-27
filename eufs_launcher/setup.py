from setuptools import setup

package_name = 'eufs_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name,
         ['launch/eufs_launcher.launch.py', 'launch/simulation.launch.py']),
        ('share/' + package_name + '/config', ['config/eufs_launcher.yaml']),
        ('share/' + package_name + '/resource', ['resource/launcher.ui']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'scipy', 'pandas'],
    zip_safe=True,
    maintainer='Angus Stewart',
    maintainer_email='siliconlad@protonmail.com',
    description='This package Launches the simulation.',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/eufs_launcher'],
)
