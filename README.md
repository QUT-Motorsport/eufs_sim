# EUFS Autonomous Simulation

ROS/Gazebo simulation packages for driverless FSAE vehicles.

![simulation](https://i.imgur.com/JZczvOr.jpg)

### Contents
1. [Install Prerequisites](#requirements)
2. [Compiling](#compiling)
3. [Using The Gui](#guiuse)
4. [Sensors](#sensors)
5. [Known issues](#issues)

## 1. Install Prerequisites <a name="requirements"></a>
- Install Ubuntu 16.04 LTS
- Install [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation)
- Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
- Put EUFS Sim inside a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Download [eufs_messages](https://gitlab.com/eufs/eufs_msgs) and put them in the same folder as `eufs_sim`
- Install ROS dependencies by navigating to the catkin workspace and doing
```bash
rosdep install -i --from-path src/
```
- Install Python dependencies:
```bash
pip install -r eufs_gazebo/requirements.txt
```

## 2. Compiling <a name="compiling"></a>

Navigate to your workspace and build the simulation:
```
cd [your-catkin-workspace]
catkin build
```

To enable ROS to find the EUFS packages you also need to run
```
source ./devel/setup.bash
```

_Note:_ source needs to be run on each new terminal you open. You can also include it in your `.bashrc` file.


## 3. Running with the GUI <a name="guiuse"></a>

Now you can finally run our kickass simulation!!
```
roslaunch eufs_launcher eufs_launcher.launch
```

You shold have something like this:

![Full Gui](https://gitlab.com/eufs/eufs_sim/wikis/uploads/eccb0eb3989c8c15a647c68f022bc3e8/LauncherFull_.png)

You can select different tracks from the dropdown menu. Then you can launch the simulation with the top-leftmost **Launch** button

The bottom-left will read in the selected image and turn it into a track, launching it immediately.
The bottom-middle will generate a random track image to `rand.png`.  You can see the image in `eufs_gazebo/randgen_imgs`.  If you want to launch it, use the bottom-left button on `rand.png`.

The bottom-left button is sensitive to a parameter called "noise" - these are randomly placed objects to the side of the track that the
car's sensors may pick up, mimicking real-world 'noise' from the environment.  By default this is off, but you can drag the slider to adjust it to whatever levels you desire.

If you don't have a good computer, stick to the Small Straights random generation preset, or perhaps Bezier if your computer is very slow.  
(Bezier tracks forgo realism for speed, whereas Small Straights keeps the track realistic, just smaller.)

An additional feature of the GUI is the ConversionTools.  As the generator creates .png files, the launcher requires .launch files, 
and important data for perception is often put into .csv format, the GUI has a converter that allows you to freely convert between file formats.
By default, files that are converted have a suffix appended to them (usually _CT) to prevent accidental overwriting of important files.
This can be turned off by checking the suffix box - the conversion process is fairly lossless, so if a file is accidentally overwritten, it
will likely behave the exact same way as the old file did.

A full manual of how to use the GUI is available ![here](https://gitlab.com/eufs/eufs_sim/wikis/Simulation/Launcher-Overview).

## 4. Additional sensors <a name="sensors"></a>
Additional sensors for testing are avilable via the `ros-kinetic-robotnik-sensor` package. Some of them are already defined in `eufs_description/robots/eufs.urdf.xarco`. You can simply commment them in and attach them appropriately to the car.


**Sensor suit of the car by default:**

- VLP16 lidar
- ZED Stereo camera
- IMU
- GPS
- odometry

An easy way to control the car is via
```
roslaunch ros_can_sim rqt_ros_can_sim.launch
```

## 5. Known issues <a name="issues"></a>
- Sometimes you might end up with 2 numpy installations at the same time. To fix this simply uninstall numpy with `pip uninstall numpy` as many times as you can. Then reinstall it with `pip install numpy` and everything should work as usual!
- **If you need to rapidly re-launch** the gui it is important that you wait a few seconds (5 should be more than enough).
If you try and do it rapidly then you may cause the gui to launch gazebo while gazebo is still shutting itself down.  This will
crash the program and won't give a satisfactory error message (it will say that it put errors in the log files, but the log files do not exist). More information can be found in the README in eufs_launcher
- **Torque controlled** car model is still experimental and should not be used unless you know what you are doing!
