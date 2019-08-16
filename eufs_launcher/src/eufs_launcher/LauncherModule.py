import os
import rospy
import rospkg
import roslaunch
import rosnode
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox, QMainWindow, QLabel, QLineEdit

from os import listdir
from os.path import isfile, join

from subprocess import Popen

from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform

from TrackGenerator import TrackGenerator as Generator

from ConversionTools import ConversionTools as Converter

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
		if "small_track.launch" in launchfiles:
			self._widget.findChild(QComboBox,"WhichTrack").addItem("small_track.launch")
		for f in launchfiles:
			if f != "small_track.launch":
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
		defaultPreset = Generator.getDefaultPreset()
		if defaultPreset in presetnames:
			self._widget.findChild(QComboBox,"WhichPreset").addItem(defaultPreset)
		for f in presetnames:
			if f != defaultPreset:
				self._widget.findChild(QComboBox,"WhichPreset").addItem(f)

		# Hook up buttons to onclick functions
		self._widget.findChild(QPushButton,"LaunchButton").clicked.connect(self.launch_button_pressed)
		self._widget.findChild(QPushButton,"GenerateButton").clicked.connect(self.generator_button_pressed)
		self._widget.findChild(QPushButton,"LoadFromImageButton").clicked.connect(self.track_from_image_button_pressed)
		self._widget.findChild(QPushButton,"ConvertButton").clicked.connect(self.convert_button_pressed)
		self._widget.findChild(QPushButton,"RenameButton").clicked.connect(self.copy_button_pressed)
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
		for f in ["launch","png","csv"]:
			self._widget.findChild(QComboBox,"ConvertFrom").addItem(f)
		for f in ["csv","png","launch","ALL"]:
			self._widget.findChild(QComboBox,"ConvertTo").addItem(f)
			
		self.updateConverterDropdown()
		self._widget.findChild(QComboBox,"ConvertFrom").currentTextChanged.connect(self.updateConverterDropdown)
		self._widget.findChild(QComboBox,"ConvertTo").currentTextChanged.connect(self.updateMidpointsBox)
		self._widget.findChild(QComboBox,"FileForConversion").currentTextChanged.connect(self.updateCopier)

		#Prep midpoints checkbox
		midpointBox = self._widget.findChild(QCheckBox,"MidpointBox")
		cvto = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		#midpointBox.setVisible(cvto=="csv" or cvto=="ALL")
		midpointBox.setChecked(True)

		#Suffix checkbox
		suffixBox = self._widget.findChild(QCheckBox,"SuffixBox")
		suffixBox.setChecked(True)

		#Track Gen full stack checkbox
		tgFullStack = self._widget.findChild(QCheckBox,"FullStackTrackGenButton")
		tgFullStack.setChecked(True)

		#Image full stack checkbox
		imFullStack = self._widget.findChild(QCheckBox,"FullStackImageButton")
		imFullStack.setChecked(True)

		#Copier full stack checkbox
		cpFullStack = self._widget.findChild(QCheckBox,"FullStackCopyButton")
		cpFullStack.setChecked(True)

		#Change label to show current selected file for the copier
		self.updateCopier()

		print("Plugin Successfully Launched!")

	def copy_button_pressed(self):
		#Copy the current file
		isFullStack = self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked()
		fileToCopyTo = self._widget.findChild(QLineEdit,"RenameFileTextbox").text()
		fileToCopyFrom = self._widget.findChild(QComboBox,"FileForConversion").currentText()
		rawName_to = fileToCopyTo.split(".")[0]
		rawName_from = fileToCopyFrom.split(".")[0]
		ending = fileToCopyFrom.split(".")[-1]

		if len(fileToCopyTo) == 0:#Don't let them create null-named files
			return

		if ending == "launch":
			#For launch files, we also need to move around the model folders
			if not os.path.exists(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_to)):
				os.mkdir(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_to))
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_from,"model.sdf")
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_to,"model.sdf")
			Converter.copyFile(path_from,path_to)

			path_from = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_from,"model.config")
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_to,"model.config")
			Converter.copyFile(path_from,path_to)
			
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch',fileToCopyFrom)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'launch',rawName_to+"."+ending)
			Converter.copyFile(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("launch","ALL",path_to,params=[self.getNoiseLevel()])

		elif ending == "png":
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs',fileToCopyFrom)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'randgen_imgs',rawName_to+"."+ending)
			Converter.copyFile(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("png","ALL",path_to,params=[self.getNoiseLevel()])

		elif ending == "csv":
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks',fileToCopyFrom)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'tracks',rawName_to+"."+ending)
			Converter.copyFile(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("csv","ALL",path_to,params=[self.getNoiseLevel()])

	def updateCopier(self):
		#Change label to show current selected file for the copier
		copyHead = self._widget.findChild(QLabel,"RenameFileHeader")
		copyHead.setText("Copy: "+self._widget.findChild(QComboBox,"FileForConversion").currentText())

	def updateMidpointsBox(self):
		#Toggle checkbox
		cvto = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		#self._widget.findChild(QCheckBox,"MidpointBox").setVisible(cvto=="csv" or cvto=="ALL")

	def updateConverterDropdown(self):
		fromType = self._widget.findChild(QComboBox,"ConvertFrom").currentText()
		allfiles = []

		if fromType == "launch":
			# Get tracks from eufs_gazebo package
			relpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch')
			launchfiles = [f for f in listdir(relpath) if isfile(join(relpath, f))]
	
			#Remove "blacklisted" files (ones that don't define tracks)
			blacklist_ = open(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/blacklist.txt'),"r")
			blacklist = [f.strip() for f in blacklist_]#remove \n
			allfiles = [f for f in launchfiles if not f in blacklist]
		elif fromType == "png":
			# Get images
			relpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs')
			allfiles = [f for f in listdir(relpath) if isfile(join(relpath, f))]
		elif fromType == "csv":
			#Get csvs
			relpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks')
			allfiles = [f for f in listdir(relpath) if isfile(join(relpath, f)) and f[-3:]=="csv"]

		#Remove old files from selector
		theSelector = self._widget.findChild(QComboBox,"FileForConversion")
		theSelector.clear()

		# Add files to selector
		for f in allfiles:
			theSelector.addItem(f)

		self.updateCopier()

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
								1 if isLaxGenerator else 0,
								1#Bezier
								])

		im = Converter.convert("xys","png",(xys,twidth,theight))

		#If full stack selected, convert into csv and launch as well
		tgFullStack = self._widget.findChild(QCheckBox,"FullStackTrackGenButton")
		if tgFullStack.isChecked():
			imgpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/rand.png')
			Converter.convert("png","ALL",imgpath,params=[self.getNoiseLevel()])

		print("Track Gen Complete!")

		im.show()

	def track_from_image_button_pressed(self):
		fname = self._widget.findChild(QComboBox,"WhichImage").currentText()
		fname_full = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+fname)
		imFullStack = self._widget.findChild(QCheckBox,"FullStackImageButton")
		if imFullStack.isChecked():
			Converter.convert("png","ALL",fname_full,params=[self.getNoiseLevel()])
		else:
			Converter.convert("png","launch",fname_full,params=[self.getNoiseLevel()])

		self.launchfileoverride = fname[:-4] + ".launch"
		self.launch_button_pressed()

	

	def getNoiseLevel(self):
		noiseLevelWidget = self._widget.findChild(QSlider,"Noisiness")
		return (1.0*(noiseLevelWidget.value()-noiseLevelWidget.minimum()))/(noiseLevelWidget.maximum()-noiseLevelWidget.minimum())

	def convert_button_pressed(self):
		fromtype = self._widget.findChild(QComboBox,"ConvertFrom").currentText()
		totype   = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		filename = self._widget.findChild(QComboBox,"FileForConversion").currentText()
		midpointWidget = self._widget.findChild(QCheckBox,"MidpointBox")
		if fromtype == "png":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+filename)
		elif fromtype == "launch":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/'+filename)
		elif fromtype == "csv":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks/'+filename)
		suffix = "_CT" if self._widget.findChild(QCheckBox,"SuffixBox").isChecked() else ""
		Converter.convert(fromtype,totype,filename,
					params=[self.getNoiseLevel(),midpointWidget.isVisible() and midpointWidget.isChecked()],conversion_suffix=suffix)

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


















