import os
import rospy
import rospkg
import roslaunch
import rosnode
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox, QMainWindow, QLabel

from os import listdir
from os.path import isfile, join

from subprocess import Popen

from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform

from TrackGenerator import TrackGenerator as Generator

from ConversionTools import ConversionTools as Convertor

class EufsLauncher(Plugin):

	def __init__(self, context):
		super(EufsLauncher, self).__init__(context)

		#Before we do anything, check if there are any nodes other than /rosout
		#And offer the option of shutting them down
		#Because often nodes get left behind due to the faulty shutdown of the launcher
		#(Not my fault, really - it's because Kinetic lacks the argument-passing feature for
		#roslaunch, meaning I have to Popen it instead, and Popen.kill doesn't successfully kill
		#all ros nodes)


		#Worst case scenario kill method: killall -9 gzserver gzclient
		#We just have to nuke it.  I've tried so much, but gazebo is incredibly unstable
		#whenever a single node gets set loose with respawn on.  Unfortunately this means
		#this gui MUST be used as a start point - trying launch something else first and then
		#loading the gui will kill it.
		#The root of the problem, I believe, is the fact that we Popen instead of rospy.roslaunch
		#the .launch file - but since we're using Kinetic and not a more recent version, the rospy.roslaunch
		#command cannot pass arguments and is thus basically useless to us.
		self.nuke_ros()

		# Give QObjects reasonable names
		self.setObjectName('EufsLauncher')

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
			dest="quiet",
			help="Put plugin in silent mode")
		args, unknowns = parser.parse_known_args(context.argv())
		if not args.quiet:
			print 'arguments: ', args
			print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which should be in the "resource" folder of this package
		ui_file = os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource', 'Launcher.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('EufsLauncherUI')
		# Show _widget.windowTitle on left-top of each plugin (when 
		# it's set in _widget). This is useful when you open multiple 
		# plugins at once. Also if you open multiple instances of your 
		# plugin at once, these lines add number to make it easy to 
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		#self._widget.setWindowTitle("Eufs Launcher v1.1.0 (Serial: " +str(context.serial_number()) + ")")

		#Resize correctly
		self._widget.setFixedWidth(1200)

		# Get tracks from eufs_gazebo package
		relpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch')
		launchfiles = [f for f in listdir(relpath) if isfile(join(relpath, f))]

		#Remove "blacklisted" files (ones that don't define tracks)
		blacklist_ = open(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/blacklist.txt'),"r")
		blacklist = [f.strip() for f in blacklist_]#remove \n
		launchfiles = [f for f in launchfiles if not f in blacklist]

		# Add Tracks to Track Selector
		for f in launchfiles:
			self._widget.findChild(QComboBox,"WhichTrack").addItem(f)

		# Get images
		relpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs')
		imagefiles = [f for f in listdir(relpath) if isfile(join(relpath, f))]

		# Add Images to Image Selector (always put rand.png first)
		if "rand.png" in imagefiles:
			self._widget.findChild(QComboBox,"WhichImage").addItem("rand.png")
		for f in imagefiles:
			if f != "rand.png" and f[-3:] == "png":
				self._widget.findChild(QComboBox,"WhichImage").addItem(f)

		#Get presets
		presetnames = Generator.getpresetnames()

		#Add Presets to Preset Selector (always put Computer Friendly first)
		if "Computer Friendly" in presetnames:
			self._widget.findChild(QComboBox,"WhichPreset").addItem("Computer Friendly")
		for f in presetnames:
			if f != "Computer Friendly":
				self._widget.findChild(QComboBox,"WhichPreset").addItem(f)

		# Hook up buttons to onclick functions
		self._widget.findChild(QPushButton,"LaunchButton").clicked.connect(self.launch_button_pressed)
		self._widget.findChild(QPushButton,"GenerateButton").clicked.connect(self.generator_button_pressed)
		self._widget.findChild(QPushButton,"LoadFromImageButton").clicked.connect(self.track_from_image_button_pressed)
		self._widget.findChild(QPushButton,"Experimentals").clicked.connect(self.experimental_button_pressed)

		#Create array of running processes (currently unused, but if you ever do process = launch.launch(node), add process here!)
		self.processes = []
		#And also an array of running launches
		self.launches = []

		# Add widget to the user interface
	        context.add_widget(self._widget)

		#Miscelaneous final initializations:
		self.hasLaunchedROS = False
		self.launchfileoverride = None
		self.popenprocess = None

		#Add experimental warning to generator
		#self._widget.findChild(QPushButton,"GenerateButton").setText("Generate Random Track\n(Experimental)")

		#Space the load track button better
		self._widget.findChild(QPushButton,"LoadFromImageButton").setText("Load Track\nFrom Image")

		#Hide generator button
		#self._widget.findChild(QPushButton,"GenerateButton").setVisible(False)

		#Hide experimental button as not currently needed
		self._widget.findChild(QPushButton,"Experimentals").setVisible(False)

		#Set up the Generator Params
		self.updatePreset()

		#Give sliders the correct range
		self.setSliderRanges()

		#Relabel the params
		self.keepParamsUpToDate()
		self.keepSlidersUpToDate()

		#Hook up sliders to function that monitors when they've been changed
		self.keepTrackOfSliderChanges()
		self.keepTrackOfPresetChanges()

		#While in the process of changing sliders, we don't want our monitor function to be rapidly firing
		#So we toggle this variable when ready
		self.ignoreSliderChanges = False

		#Setup Lax Generation button
		laxButton = self._widget.findChild(QCheckBox,"LaxCheckBox")
		laxButton.setChecked(self.LAX_GENERATION)
		
		#Setup Conversion Tools dropdowns
		for f in [".png",".launch",".csv"]:
			self._widget.findChild(QComboBox,"ConvertFrom").addItem(f)
			

		print("Plugin Successfully Launched!")

	def updatePreset(self):
		which = self._widget.findChild(QComboBox,"WhichPreset").currentText()
		presetData = Generator.getpreset(which)
		self.MIN_STRAIGHT   = presetData[0]
		self.MAX_STRAIGHT   = presetData[1]
		self.MIN_CTURN      = presetData[2]
		self.MAX_CTURN      = presetData[3]
		self.MIN_HAIRPIN    = presetData[4]*2
		self.MAX_HAIRPIN    = presetData[5]*2
		self.HAIRPIN_PAIRS  = presetData[6]
		self.MAX_LENGTH     = presetData[7]
		self.LAX_GENERATION = presetData[8]
		self._widget.findChild(QCheckBox,"LaxCheckBox").setChecked(self.LAX_GENERATION)

	def keepTrackOfPresetChanges(self):
		self._widget.findChild(QComboBox,"WhichPreset") .currentTextChanged.connect(self.presetChanged)

	def presetChanged(self):
		self.ignoreSliderChanges = True
		self.updatePreset()
		self.keepParamsUpToDate()
		self.keepSlidersUpToDate()
		self.ignoreSliderChanges = False

	def keepTrackOfSliderChanges(self):
		self._widget.findChild(QSlider,"Param_MIN_STRAIGHT") .valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_MAX_STRAIGHT") .valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_MIN_CTURN")    .valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_MAX_CTURN")    .valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_MIN_HAIRPIN")  .valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_MAX_HAIRPIN")  .valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_HAIRPIN_PAIRS").valueChanged.connect(self.sliderChanged)
		self._widget.findChild(QSlider,"Param_MAX_LENGTH")   .valueChanged.connect(self.sliderChanged)
		
	def sliderChanged(self):
		if self.ignoreSliderChanges: return
		self.keepVariablesUpToDate()
		self.keepParamsUpToDate()

	def keepParamsUpToDate(self):
		#This function keeps the labels next to the sliders up to date with the actual values
		self._widget.findChild(QLabel,"Label_MIN_STRAIGHT") .setText("MIN_STRAIGHT: "  + str(self.MIN_STRAIGHT))
		self._widget.findChild(QLabel,"Label_MAX_STRAIGHT") .setText("MAX_STRAIGHT: "  + str(self.MAX_STRAIGHT))
		self._widget.findChild(QLabel,"Label_MIN_CTURN")    .setText("MIN_CTURN: "     + str(self.MIN_CTURN))
		self._widget.findChild(QLabel,"Label_MAX_CTURN")    .setText("MAX_CTURN: "     + str(self.MAX_CTURN))
		self._widget.findChild(QLabel,"Label_MIN_HAIRPIN")  .setText("MIN_HAIRPIN: "   + str((self.MIN_HAIRPIN/2.0)))
		self._widget.findChild(QLabel,"Label_MAX_HAIRPIN")  .setText("MAX_HAIRPIN: "   + str((self.MAX_HAIRPIN/2.0)))
		self._widget.findChild(QLabel,"Label_HAIRPIN_PAIRS").setText("HAIRPIN_PAIRS: " + str(self.HAIRPIN_PAIRS))
		self._widget.findChild(QLabel,"Label_MAX_LENGTH")   .setText("MAX_LENGTH: "    + str(self.MAX_LENGTH))

	def keepSlidersUpToDate(self):
		self.setSliderValue("Param_MIN_STRAIGHT",self.MIN_STRAIGHT)
		self.setSliderValue("Param_MAX_STRAIGHT",self.MAX_STRAIGHT)
		self.setSliderValue("Param_MIN_CTURN",self.MIN_CTURN)
		self.setSliderValue("Param_MAX_CTURN",self.MAX_CTURN)
		self.setSliderValue("Param_MIN_HAIRPIN",self.MIN_HAIRPIN)
		self.setSliderValue("Param_MAX_HAIRPIN",self.MAX_HAIRPIN)
		self.setSliderValue("Param_HAIRPIN_PAIRS",self.HAIRPIN_PAIRS)
		self.setSliderValue("Param_MAX_LENGTH",self.MAX_LENGTH)

	def keepVariablesUpToDate(self):
		self.MIN_STRAIGHT = self.getSliderValue("Param_MIN_STRAIGHT")
		self.MAX_STRAIGHT = self.getSliderValue("Param_MAX_STRAIGHT")
		self.MIN_CTURN = self.getSliderValue("Param_MIN_CTURN")
		self.MAX_CTURN = self.getSliderValue("Param_MAX_CTURN")
		self.MIN_HAIRPIN = self.getSliderValue("Param_MIN_HAIRPIN")
		self.MAX_HAIRPIN = self.getSliderValue("Param_MAX_HAIRPIN")
		self.HAIRPIN_PAIRS = self.getSliderValue("Param_HAIRPIN_PAIRS")
		self.MAX_LENGTH = self.getSliderValue("Param_MAX_LENGTH")

	def setSliderRanges(self):
		#This function keeps slider locations up to date with the actual values
		#Note on ranges: 
		#		Straights are between 0 and 150
		#		Turns are between 0 and 50
		#		Hairpins are between 0 and 20, and have half-step rather than integer step (hence scaling by 2s)
		#		Hairpin pairs are between 0 and 5
		#               Max Length is between 200 and 2000
		minstraight = 0
		maxstraight = 150
		minturn = 0
		maxturn = 50
		minhairpin = 0
		maxhairpin = 20
		minhairpinpairs = 0
		maxhairpinpairs = 5
		minmaxlength = 200
		maxmaxlength = 2000
		self.setSliderData("Param_MIN_STRAIGHT",minstraight,maxstraight)
		self.setSliderData("Param_MAX_STRAIGHT",minstraight,maxstraight)
		self.setSliderData("Param_MIN_CTURN",minturn,maxturn)
		self.setSliderData("Param_MAX_CTURN",minturn,maxturn)
		self.setSliderData("Param_MIN_HAIRPIN",minhairpin*2,maxhairpin*2)
		self.setSliderData("Param_MAX_HAIRPIN",minhairpin*2,maxhairpin*2)
		self.setSliderData("Param_HAIRPIN_PAIRS",minhairpinpairs,maxhairpinpairs)
		self.setSliderData("Param_MAX_LENGTH",minmaxlength,maxmaxlength)
		
	def getSliderValue(self,slidername):
		slider = self._widget.findChild(QSlider,slidername)
		return slider.value()

	def setSliderValue(self,slidername,sliderval):
		slider = self._widget.findChild(QSlider,slidername)
		slider.setValue(sliderval)

	def setSliderData(self,slidername,slidermin,slidermax):
		slider = self._widget.findChild(QSlider,slidername)
		slider.setMinimum(slidermin)
		slider.setMaximum(slidermax)
		

	def experimental_button_pressed(self):
		self._widget.findChild(QPushButton,"GenerateButton").setVisible(True)
		

	def generator_button_pressed(self):
		print("Generating Track...")

		isLaxGenerator = self._widget.findChild(QCheckBox,"LaxCheckBox").isChecked()
		

		xys,twidth,theight = Generator.generate([	self.MIN_STRAIGHT,self.MAX_STRAIGHT,
								self.MIN_CTURN,self.MAX_CTURN,
								self.MIN_HAIRPIN/2,self.MAX_HAIRPIN/2,
								self.HAIRPIN_PAIRS,
								self.MAX_LENGTH,
								1 if isLaxGenerator else 0
								])

		im = Convertor.convert("xys","png",(xys,twidth,theight))

		print("Track Gen Complete!")

		im.show()
		#self.launchfileoverride = GENERATED_FILENAME + ".launch"
		#self.launch_button_pressed()

	def track_from_image_button_pressed(self):
		fname = self._widget.findChild(QComboBox,"WhichImage").currentText()
		fname_full = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+fname)
		self.create_track_from_image(Image.open(fname_full),fname[:-4])#[:-4] to split off .png, looks like an emoji...
		self.launchfileoverride = fname[:-4] + ".launch"
		self.launch_button_pressed()

	def create_track_from_image(self,im,GENERATED_FILENAME):
		#This is a fairly intensive process - we need:
		#	to put %FILENAME%.launch in eufs_gazebo/launch
		#	to put %FILENAME%.world in eufs_gazebo/world
		#	to put %FILENAME%/model.config and %FILENAME%/model.sdf in eufs_description/models

		#Our template files are stored in eufs_launcher/resource as:
		#	randgen_launch_template
		#       randgen_world_template
		#	randgen_model_template/model.config
		#	randgen_model_template/model.sdf

		#.launch:
		launch_template = open(os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource/randgen_launch_template'),"r")
		#params = %FILLNAME% %PLACEX% %PLACEY% %PLACEROTATION%
		launch_merged = "".join(launch_template)
		launch_merged = GENERATED_FILENAME.join(launch_merged.split("%FILLNAME%"))

		def xCoordTransform(x):
			return x-50
		def yCoordTransform(y):
			return y-50
		def isCarColor(x):
			return x[:-1]==self.carcolor[:-1]
		def rotationTransform(x):
			return 2*math.pi*((x-1)/254.0)#we never allow alpha to equal 0

		#Get PLACEX,PLACEY (look for (0,255,0,a))
		pixels = im.load()
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				p = pixels[i,j]
				if isCarColor(p):
					launch_merged = str(xCoordTransform(i)).join(launch_merged.split("%PLACEX%"))
					launch_merged = str(yCoordTransform(j)).join(launch_merged.split("%PLACEY%"))
					launch_merged = str(rotationTransform(p[3])).join(launch_merged.split("%PLACEROTATION%"))

		launch_out = open(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/'+GENERATED_FILENAME+".launch"),"w")
		launch_out.write(launch_merged)
		launch_out.close()

		#.world:
		world_template = open(os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource/randgen_world_template'),"r")
		#params = %FILLNAME%
		world_merged = "".join(world_template)
		world_merged = GENERATED_FILENAME.join(world_merged.split("%FILLNAME%"))

		world_out = open(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'worlds',GENERATED_FILENAME+".world"),"w")
		world_out.write(world_merged)
		world_out.close()

		#model:
		#First we create the folder
		folderpath = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',GENERATED_FILENAME)
		if not os.path.exists(folderpath):
			os.mkdir(folderpath)
		else:
			print("Overwrote old " + GENERATED_FILENAME)

		#Now let's do the .config as its easiest
		config_template = open(os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource/randgen_model_template/model.config'),"r")
		#params = %FILLNAME%
		config_merged = "".join(config_template)
		config_merged = GENERATED_FILENAME.join(config_merged.split("%FILLNAME%"))

		config_out = open(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',GENERATED_FILENAME,"model.config"),"w")
		config_out.write(config_merged)
		config_out.close()

		#Now the real meat of this, the .sdf
		sdf_template = open(os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource/randgen_model_template/model.sdf'),"r")
		#params = %FILLNAME% %FILLDATA%
		#ModelParams = %PLACEX% %PLACEY% %MODELNAME% %FILLCOLLISION% %LINKNUM%
		sdf_merged = "".join(sdf_template)
		sdf_splitagain = sdf_merged.split("$===$")

		#sdf_splitagain:
		#	0: Main body of sdf file
		#	1: Outline of mesh visual
		#	2: Outline of mesh collision data
		#	3: Noisecube collision data, meant for noise as a low-complexity collision to prevent falling out the world
		sdf_main = sdf_splitagain[0]
		sdf_model = sdf_splitagain[3].join(sdf_splitagain[1].split("%FILLCOLLISION%"))
		sdf_model_with_collisions = sdf_splitagain[2].join(sdf_splitagain[1].split("%FILLCOLLISION%"))

		sdf_main = GENERATED_FILENAME.join(sdf_main.split("%FILLNAME%"))

		sdf_blueconemodel = "model://eufs_description/meshes/cone_blue.dae".join(sdf_model_with_collisions.split("%MODELNAME%"))
		sdf_yellowconemodel = "model://eufs_description/meshes/cone_yellow.dae".join(sdf_model_with_collisions.split("%MODELNAME%"))

		#Now let's load in the noise priorities
		noisefiles = open(os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource/noiseFiles.txt'),"r")
		noisefiles = ("".join(noisefiles)).split("$===$")[1].strip().split("\n")
		noiseweightings_ = [line.split("|") for line in noisefiles]
		noiseweightings  = [(float(line[0]),line[1]) for line in noiseweightings_]

		def getRandomNoiseModel():
			randval = uniform(0,100)
			for a in noiseweightings:
				if a[0]>randval:
					return a[1]
			return "model://eufs_description/meshes/NoiseCube.dae"
		
		#Let's place all the models!
		self.linknum = -1
		def putModelAtPosition(mod,x,y):
			self.linknum+=1
			return str(self.linknum).join( \
				str(xCoordTransform(x)).join( \
					str(yCoordTransform(y)).join( \
						mod.split("%PLACEY%")).split("%PLACEX%")).split("%LINKNUM%"))

		sdf_allmodels = ""
		noiseLevel = self.getNoiseLevel()
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				p = pixels[i,j]
				if p == self.conecolor:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_yellowconemodel,i,j)
				elif p == self.conecolor2:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_blueconemodel,i,j)
				elif p == self.noisecolor and uniform(0,1)<noiseLevel:
					sdf_noisemodel = getRandomNoiseModel().join(sdf_model_with_collisions.split("%MODELNAME%"))
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_noisemodel,i,j)

		sdf_main = sdf_allmodels.join(sdf_main.split("%FILLDATA%"))

		sdf_out = open(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',GENERATED_FILENAME,"model.sdf"),"w")
		sdf_out.write(sdf_main)
		sdf_out.close()

	def getNoiseLevel(self):
		noiseLevelWidget = self._widget.findChild(QSlider,"Noisiness")
		return (1.0*(noiseLevelWidget.value()-noiseLevelWidget.minimum()))/(noiseLevelWidget.maximum()-noiseLevelWidget.minimum())

	def launch_button_pressed(self):
		if self.hasLaunchedROS:
			#Don't let people press launch twice
			#There's not really any reason why not to, but it's "undefined behavior"
			return
		self.hasLaunchedROS = True
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		print("--------------------------")
		print("\t\t\tLaunching Nodes...")
		trackToLaunch = self.launchfileoverride #if we have set a specific file to run regardless of selected file
		self.launchfileoverride = None          #such as when we use the random generator
		if not trackToLaunch:
			trackToLaunch = self._widget.findChild(QComboBox,"WhichTrack").currentText()
		print("Launching " + trackToLaunch)
		noiseLevel = self.getNoiseLevel()
		print("With Noise Level: " + str(noiseLevel))
		
		controlMethod = "controlMethod:=speed"
		if self._widget.findChild(QRadioButton,"SpeedRadio").isChecked():
			print("With Speed Controls")
			controlMethod = "controlMethod:=speed"
		elif self._widget.findChild(QRadioButton,"TorqueRadio").isChecked():
			print("With Torque Controls")
			controlMethod = "controlMethod:=torque"
		
		#Ideally we would use the commented out 'launch' lines, but the ROSLaunchParent API does not allow sending arguments in Kinetic >:(
		#So we have to settle for launching this process using Python's Popen instead of the rospy API functions.
		if self.popenprocess:
			self.process.kill()
		self.popenprocess = Popen(["roslaunch",os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch', trackToLaunch),controlMethod])
		#launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch', trackToLaunch)])
		#launch.start()
		#self.launches.append(launch)


		if self._widget.findChild(QCheckBox,"VisualisatorCheckbox").isChecked():
			print("And With LIDAR Data Visualisator.")
			launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('eufs_description'), "launch/visualisator.launch")])
			launch.start()
			self.launches.append(launch)
		print("--------------------------")

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		print("Shutdown Engaged...")
		#(Stop all processes)
		for p in self.processes:
			p.stop()

		for l in self.launches:
			l.shutdown()

		#Trying to kill here will make a horrible bug
		#no idea why.  You'll want: "killall -9 gzserver gzclient" to fix it
		#if self.popenprocess != None:
		#	self.popenprocess.kill()
		#	self.popenprocess = None

		#NUKE IT! (seriously just nuke it)
		self.nuke_ros()

		#print("Shutdown Complete!")

	def nuke_ros(self):
		#Try to kill as much as possible
		#Burn it all to the ground
		Popen(["killall","-9","gzserver"])
		Popen(["killall","-9","gzclient"])
		extranodes = rosnode.get_node_names()
		extranodes.remove('/rosout')
		extranodes = [f for f in extranodes if f[:17] != '/rqt_gui_py_node_'] #remove the gui's node from kill list
		for badnode in extranodes:
			Popen(["rosnode","kill",badnode])


	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog


















