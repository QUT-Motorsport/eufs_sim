# EUFS Launcher

The information here can be found at the [team wiki](https://gitlab.com/eufs/resources/wikis/Simulator/Image-To-Track-Conversion), which will be kept up-to-date.

## Plugins:

### LauncherModule
Provides the interface in order to launch the simulation. Controls the package used to load the simulation, the track to load and other vehicle control parameters.

#### Parameters:

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| config | string | config/eufs_launcher | File name of yaml file containing launcher defaults. |
| config_loc | string | eufs_launcher | Directory name of yaml file to be used. |
| gui | bool | True | Condition to launch the GUI. | 

## Nodes:
There is only one ros node associated with this package called `eufs_launcher`. This is the Launcher GUI Node. This node does not communicate with other nodes.

## How To Use the GUI

Basic instructions can be found in the main repository README.  Here lie the advanced usage notes.

### Editing the GUI's UI

The File for editing the Launcher GUI's layout can be found in `eufs_launcher/resource` in the `launcher.ui` file.
For adding additional options on launch, `eufs_launcher/src/eufs_launcher/LauncherModule.py` can be edited.
However, at this point it is recommended to let the infrastructure team handle it.

#### Noise

Noise is represented by `cyan` on an image.  Randomly generated images will have approximately 1% of the image covered with noise.
(more precisely, each pixel not between cones has a 1% chance of being noise).  The GUI's noise slider has no effect on pixel placement
in randomly generated images.

When the launcher reads an image file and converts it into a track, then it takes into account the noise slider.  
If the noise slider is x% full, then every noise pixel has an x% chance of being realized in the final product.
(so at 0% there is no noise and at 100% every noise pixel has a noise object on it).

Noise objects are chosen from `eufs_track_generator/resource/noiseFiles.txt` - feel free to add your own and edit the weights!
Full instructions are how to do so are inside the file itself.
You may wish to add a completely new model - for that, you need a .dae file to be placed in `eufs_description/meshes`.  If you have Blender,
these are easy to procure - Blender has an export-to-.dae option!  Even if you don't know how to use Blender to create objects, many people
online have free resources for Blender, so you can load those up and then export.
