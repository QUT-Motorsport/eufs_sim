# eufs_tracks

There are a number of wiki pages describing the design and usage of eufs_tracks, they can be found in the following locations:
- [Track Models](https://gitlab.com/eufs/eufs_sim/-/wikis/Track-models)
- [Random Track Generation](https://gitlab.com/eufs/eufs_sim/-/wikis/Simulation/Random-Track-Generation)
- [Track Image File Format Specification](https://gitlab.com/eufs/eufs_sim/-/wikis/Simulation/Track-Image-File-Format-Specification)
- [Image To Track Conversion](https://gitlab.com/eufs/eufs_sim/-/wikis/Simulation/Image-To-Track-Conversion)

## TrackGeneratorModule
Provides the interface in order to generate random tracks according to a set of generation parameters. This also allows the user to convert between different track file types.

### Parameters:
There are no launch parameters for this GUI. It is not configurable.

### ROS 2 Nodes:

There is only one ROS 2 node associated with this package called `eufs_track_generator`. This is the Track Generator GUI Node. This node does not communicate with other nodes.

An example of how to launch the track generator node can be found [here](./launch/eufs_track_generator.launch.py).

### GUI Components

#### Track Generator

| Label | Type | Default | Purpose |
| ----- | ---- | ------- | ------- |
| MIN_STRAIGHT          | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 5               | Minimum straight length (metres). |
| MAX_STRAIGHT          | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 40              | Maximum straight length (metres). |
| MIN_CTURN             | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 10              | Minimum radius of generated cturn segments (metres). |
| MAX_CTURN             | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 25              | Maximum radius of generated cturn segments (metres). |
| MIN_HAIRPIN           | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 4.5             | Minimum radius of generated hairpin segments (metres). |
| MAX_HAIRPIN           | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 10.0            | Maximum radius of generated hairpin segments (metres). |
| HAIRPIN_PAIRS         | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 3               | Maximum amount of back-to-back pairs of turns in a hairpin. |
| MAX_LENGTH            | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 700             | Maximum track length (metres). |
| Preset                | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | Small Straights | Choice of preset parameter combinations to generate specific track characteristics. |
| Lax Generation        | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | true            | Whether generation parameters are rigidly followed by Generator, if true, only failure condition is self-intersection. |
| Full Stack            | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | true            | Whether generated tracks are converted into all supported file formats. |
| TRACK_WIDTH           | [QSlider](https://doc.qt.io/qt-5/qslider.html)         | 3.5             | Distance of cones from track centre (metres)
| Generate Random Track | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -               | Generate track with current parameters. |

#### Track Converter

| Label | Type | Default | Purpose |
| ----- | ---- | ------- | ------- |
| From               | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | launch              | Specifies the file format of the track to convert. |
| To                 | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | csv                 | Specifies the file format we wish to convert to. |
| Filename           | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | acceleration.launch | File to be converted. |
| Midpoints          | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | true                | Whether track should contain midpoint info when converting to csv. |
| Suffix             | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | true                | Whether to add the "_CT" suffix to converted track name. |
| Convert            | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -                   | Convert the track with specified conversion settings. |
| Copy: `TRACKNAME`  | [QLabel](https://doc.qt.io/qt-5/qlabel.html)           | acceleration.launch | Track to copy (track specified in Filename QComboBox). |
| Create Copy called | [QLineEdit](https://doc.qt.io/qt-5/qlineedit.html)     | -                   | Name of new file. |
| Full Stack         | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | true                | Whether track should be copied to all supported file formats. |
| Copy               | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -                   | Copy the track with specified copying settings. |

### Editing the GUI's UI

The track generator GUI can be edited using [track_generator.ui](./resource/track_generator.ui).
For adding additional options on launch see the [TrackGeneratorModule](./src/eufs_tracks/TrackGeneratorModule.py).

### Image-to-Track

#### Relevant Locations

Images are expected to be in the [image](./image) folder in `.png` format.

When creating a track from an image, the launcher creates files based on templates found in [resource](./resource) -
editing them will change the resultant track.

#### Creating One's Own Image

At this time, the convertor only looks at these specific pixel types [given in RGBA]:

```
Yellow Cones: (255,0,255,255) "Magenta" [Yellow was already taken]
Blue Cones: (0,0,255,255) "Blue"
Car: (0,255,0,a) "Green"
Noise: (0,255,255,255) "Cyan"
Background: (255,255,255,255) "White"
```

All colors not listed here will be treated as `Background`.

However, the image generator also outputs images with other colors - here is a full list from `TrackGeneratorModule.py`:

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
so although they are not currently used it may be worth adding them to your `.png`s for future-proofing.

See [skidpad.png](./image/skidpad.png) for an example image.

The car color can have any "a" from 1 to 255 (note: NOT 0) - it is converted into an angle:
```2*math.pi*((x-1)/254.0)```
which will be the direction the front of the car is facing.

#### The Generated Track

When loading an image with name `[name].png`, the track generator will create files `[name].launch` and `[name].world` in their relevant eufs_tracks folders.
It will also create a folder `[name]` in [models](./models), which has two files in it, `model.sdf` and `model.config`.
If any of these files already exist, it will overwrite!  This is by design - loading the same image will give the same track.

Note that this all applies to the file `rand.png` as well, which is the output for random track generation!