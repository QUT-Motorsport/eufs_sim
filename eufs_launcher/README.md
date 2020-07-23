# EUFS GUI Launcher

The information here can be found at the [team wiki](https://gitlab.com/eufs/resources/wikis/Simulator/Image-To-Track-Conversion), which will be kept up-to-date.

## How To Use

Basic instructions can be found in the main repository README.  Here lie the advanced usage notes.

### Editing the GUI's UI

If you don't like the layout, in `eufs_launcher/resource` there exists a file `Launcher.ui`.
If you want to do basic layout editing, it should be self explanatory how to edit this file.
If you want to add whole new options on startup, you will want to direct yourself to `eufs_launcher/src/eufs_launcher/LauncherModule.py`,
but if you're at that point, you might as well just get in touch with the simulation team and let them handle it.

### Image-to-Track

#### Relevant Locations

Images are expected to be in the `eufs_gazebo/randgen_imgs` folder in `.png` format.

When creating a track from an image, the launcher creates files based on templates found in `eufs_launcher/resource` -
editing them will change the resultant track.

#### Creating One's Own Image

[Take a look at minimalist.png for an example of an image using only the necessary colors]

At this time, the convertor only looks at these specific pixel types [given in RGBA]:

```
Yellow Cones: (255,0,255,255) "Magenta" [Yellow was already taken]
Blue Cones: (0,0,255,255) "Blue"
Car: (0,255,0,a) "Green"
Noise: (0,255,255,255) "Cyan"
Background: (255,255,255,255) "White"
```

All colors not listed here will be treated as `Background`

However the image generator also outputs images with other colors - here is a full list from `LauncherModule.py`:

```python
self.noisecolor = (0,255,255,255)       #cyan
self.bgcolor = (255,255,255,255)        #white
self.conecolor = (255,0,255,255)        #magenta
self.conecolor2 = (0,0,255,255)         #blue
self.carcolor = (0,255,0,255)           #green
self.trackcenter = (0,0,0,255)          #black
self.trackinner = (255,0,0,255)         #red
self.trackouter = (255,255,0,255)       #yellow
```

Future updates to the image converter may decide to make use of additional data using these colors,
so although they are not currently used it may be worth adding them to your `.png`s for future-proofing
(you can see how they are used by taking a look at `rand.png`, which is the output file for randomly generated images).

The car color can have any "a" from 1 to 255 (note: NOT 0) - it is converted into an angle:
```2*math.pi*((x-1)/254.0)```
which will be the direction the front of the car is facing.

#### The Generated Track

When loading an image with name `[name].png`, the launcher will create files `[name].launch` and `[name].world` in their relevant eufs_gazebo folders.
It will also create a folder `[name]` in `eufs_description/models`, which has two files in it, `model.sdf` and `model.config`.
If any of these files already exist, it will overwrite!  This is by design - loading the same image will give the same track, with the exception
of the precise placement of noise objects which depend on the noise level selected in the gui and also random chance (unless at max or min noise).
So if you want to save the precise noise data (which will be in the aforementioned `model.sdf`) then you should copy that file before loading the image again.

Note that this all applies to the file `rand.png` as well, which is the output for random track generation!

#### Noise

Noise is represented by `cyan` on an image.  Randomly generated images will have approximately 1% of the image covered with noise.
(more precisely, each pixel not between cones has a 1% chance of being noise).  The GUI's noise slider has no effect on pixel placement
in randomly generated images.

When the launcher reads an image file and converts it into a track, then it takes into account the noise slider.  
If the noise slider is x% full, then every noise pixel has an x% chance of being realized in the final product.
(so at 0% there is no noise and at 100% every noise pixel has a noise object on it).

Noise objects are chosen from `eufs_launcher/resource/noiseFiles.txt` - feel free to add your own and edit the weights!
Full instructions are how to do so are inside the file itself.
You may wish to add a completely new model - for that, you need a .dae file to be placed in `eufs_description/meshes`.  If you have Blender,
these are easy to procure - Blender has an export-to-.dae option!  Even if you don't know how to use Blender to create objects, many people
online have free resources for Blender, so you can load those up and then export.
