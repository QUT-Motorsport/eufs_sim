from setuptools import setup

package_name = 'ros_can_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name + '/resource', [
            'resource/EUFSRobotSteeringGUI.ui',
            'resource/RosCanSimGUI.ui'
        ]),
        ('share/' + package_name, ['plugin.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niklas Burggraaff',
    maintainer_email='niklasburggraaff@gmail.com',
    description='Package which currently just has rqt GUIs',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/eufs_robot_steering_gui', 'scripts/ros_can_sim_gui'],
)
