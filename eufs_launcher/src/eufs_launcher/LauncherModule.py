import os
import rospy
import rospkg
import roslaunch
import rosnode
import math
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox, QMainWindow, QLabel, QLineEdit, QApplication

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
		self.main_ui_file = os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource', 'Launcher.ui')
		self.sketcher_ui_file = os.path.join(rospkg.RosPack().get_path('eufs_launcher'), 'resource', 'Sketcher.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(self.main_ui_file, self._widget)
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

		self.load_track_and_images()

		#Get presets
		presetnames = Generator.get_preset_names()

		#Add Presets to Preset Selector (always put Computer Friendly first)
		defaultPreset = Generator.get_default_preset()
		if defaultPreset in presetnames:
			self._widget.findChild(QComboBox,"WhichPreset").addItem(defaultPreset)
		for f in presetnames:
			if f != defaultPreset:
				self._widget.findChild(QComboBox,"WhichPreset").addItem(f)

		#Hook up buttons to onclick functions
		self._widget.findChild(QPushButton,"LaunchButton").clicked.connect(self.launch_button_pressed)
		self._widget.findChild(QPushButton,"GenerateButton").clicked.connect(self.generator_button_pressed)
		self._widget.findChild(QPushButton,"LoadFromImageButton").clicked.connect(self.track_from_image_button_pressed)
		self._widget.findChild(QPushButton,"ConvertButton").clicked.connect(self.convert_button_pressed)
		self._widget.findChild(QPushButton,"RenameButton").clicked.connect(self.copy_button_pressed)
		self._widget.findChild(QPushButton,"Experimentals").clicked.connect(self.experimental_button_pressed)
		self._widget.findChild(QPushButton,"SketcherButton").clicked.connect(self.sketcher_button_pressed)


		#Create array of running processes (currently unused, but if you ever do process = launch.launch(node), add process here!)
		self.processes = []
		#And also an array of running launches
		self.launches = []
		#And array of running popens
		self.popens = []

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

		#Hide track draw button as not currently working
		self._widget.findChild(QPushButton,"SketcherButton").setVisible(False)

		#Set up the Generator Params
		self.update_preset()

		#Give sliders the correct range
		self.set_slider_ranges()

		#Relabel the params
		self.keep_params_up_to_date()
		self.keep_sliders_up_to_date()

		#Hook up sliders to function that monitors when they've been changed
		self.keep_track_of_slider_changes()
		self.keep_track_of_preset_changes()

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
			
		self.update_converter_dropdown()
		self._widget.findChild(QComboBox,"ConvertFrom").currentTextChanged.connect(self.update_converter_dropdown)
		self._widget.findChild(QComboBox,"ConvertTo").currentTextChanged.connect(self.update_midpoints_box)
		self._widget.findChild(QComboBox,"FileForConversion").currentTextChanged.connect(self.update_copier)

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
		self.update_copier()

		#Get uuid
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)

	def tell_launchella(self,what):
		self._widget.findChild(QLabel,"UserFeedbackLabel").setText(what)
		QApplication.processEvents() 

	def sketcher_button_pressed(self):
		rospy.logerr("Opening sketcher...")
		loadUi(self.sketcher_ui_file, self._widget)

	def load_track_and_images(self):
		# Clear the dropdowns
		self._widget.findChild(QComboBox,"WhichTrack").clear()
		self._widget.findChild(QComboBox,"WhichImage").clear()
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

	def copy_button_pressed(self):
		self.tell_launchella("Copying...")
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
			Converter.copy_file(path_from,path_to)

			path_from = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_from,"model.config")
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',rawName_to,"model.config")
			Converter.copy_file(path_from,path_to)
			
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch',fileToCopyFrom)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'launch',rawName_to+"."+ending)
			Converter.copy_file(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("launch","ALL",path_to,params=[self.get_noise_level()])

		elif ending == "png":
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs',fileToCopyFrom)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'randgen_imgs',rawName_to+"."+ending)
			Converter.copy_file(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("png","ALL",path_to,params=[self.get_noise_level()])

		elif ending == "csv":
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks',fileToCopyFrom)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'tracks',rawName_to+"."+ending)
			Converter.copy_file(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("csv","ALL",path_to,params=[self.get_noise_level()])
		self.load_track_and_images()
		self.tell_launchella("Copy Succeeded!")

	def update_copier(self):
		#Change label to show current selected file for the copier
		copyHead = self._widget.findChild(QLabel,"RenameFileHeader")
		copyHead.setText("Copy: "+self._widget.findChild(QComboBox,"FileForConversion").currentText())

	def update_midpoints_box(self):
		#Toggle checkbox
		cvto = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		#self._widget.findChild(QCheckBox,"MidpointBox").setVisible(cvto=="csv" or cvto=="ALL")

	def update_converter_dropdown(self):
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

		self.update_copier()

	def update_preset(self):
		which = self._widget.findChild(QComboBox,"WhichPreset").currentText()
		presetData = Generator.get_preset(which)
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

	def keep_track_of_preset_changes(self):
		self._widget.findChild(QComboBox,"WhichPreset") .currentTextChanged.connect(self.preset_changed)

	def preset_changed(self):
		self.ignoreSliderChanges = True
		self.update_preset()
		self.keep_params_up_to_date()
		self.keep_sliders_up_to_date()
		self.ignoreSliderChanges = False

	def keep_track_of_slider_changes(self):
		self._widget.findChild(QSlider,"Param_MIN_STRAIGHT") .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_STRAIGHT") .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MIN_CTURN")    .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_CTURN")    .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MIN_HAIRPIN")  .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_HAIRPIN")  .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_HAIRPIN_PAIRS").valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_LENGTH")   .valueChanged.connect(self.slider_changed)
		
	def slider_changed(self):
		if self.ignoreSliderChanges: return
		self.keep_variables_up_to_date()
		self.keep_params_up_to_date()

	def keep_params_up_to_date(self):
		#This function keeps the labels next to the sliders up to date with the actual values
		self._widget.findChild(QLabel,"Label_MIN_STRAIGHT") .setText("MIN_STRAIGHT: "  + str(self.MIN_STRAIGHT))
		self._widget.findChild(QLabel,"Label_MAX_STRAIGHT") .setText("MAX_STRAIGHT: "  + str(self.MAX_STRAIGHT))
		self._widget.findChild(QLabel,"Label_MIN_CTURN")    .setText("MIN_CTURN: "     + str(self.MIN_CTURN))
		self._widget.findChild(QLabel,"Label_MAX_CTURN")    .setText("MAX_CTURN: "     + str(self.MAX_CTURN))
		self._widget.findChild(QLabel,"Label_MIN_HAIRPIN")  .setText("MIN_HAIRPIN: "   + str((self.MIN_HAIRPIN/2.0)))
		self._widget.findChild(QLabel,"Label_MAX_HAIRPIN")  .setText("MAX_HAIRPIN: "   + str((self.MAX_HAIRPIN/2.0)))
		self._widget.findChild(QLabel,"Label_HAIRPIN_PAIRS").setText("HAIRPIN_PAIRS: " + str(self.HAIRPIN_PAIRS))
		self._widget.findChild(QLabel,"Label_MAX_LENGTH")   .setText("MAX_LENGTH: "    + str(self.MAX_LENGTH))

	def keep_sliders_up_to_date(self):
		self.set_slider_value("Param_MIN_STRAIGHT",self.MIN_STRAIGHT)
		self.set_slider_value("Param_MAX_STRAIGHT",self.MAX_STRAIGHT)
		self.set_slider_value("Param_MIN_CTURN",self.MIN_CTURN)
		self.set_slider_value("Param_MAX_CTURN",self.MAX_CTURN)
		self.set_slider_value("Param_MIN_HAIRPIN",self.MIN_HAIRPIN)
		self.set_slider_value("Param_MAX_HAIRPIN",self.MAX_HAIRPIN)
		self.set_slider_value("Param_HAIRPIN_PAIRS",self.HAIRPIN_PAIRS)
		self.set_slider_value("Param_MAX_LENGTH",self.MAX_LENGTH)

	def keep_variables_up_to_date(self):
		self.MIN_STRAIGHT = self.get_slider_value("Param_MIN_STRAIGHT")
		self.MAX_STRAIGHT = self.get_slider_value("Param_MAX_STRAIGHT")
		self.MIN_CTURN = self.get_slider_value("Param_MIN_CTURN")
		self.MAX_CTURN = self.get_slider_value("Param_MAX_CTURN")
		self.MIN_HAIRPIN = self.get_slider_value("Param_MIN_HAIRPIN")
		self.MAX_HAIRPIN = self.get_slider_value("Param_MAX_HAIRPIN")
		self.HAIRPIN_PAIRS = self.get_slider_value("Param_HAIRPIN_PAIRS")
		self.MAX_LENGTH = self.get_slider_value("Param_MAX_LENGTH")

	def set_slider_ranges(self):
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
		self.set_slider_data("Param_MIN_STRAIGHT",minstraight,maxstraight)
		self.set_slider_data("Param_MAX_STRAIGHT",minstraight,maxstraight)
		self.set_slider_data("Param_MIN_CTURN",minturn,maxturn)
		self.set_slider_data("Param_MAX_CTURN",minturn,maxturn)
		self.set_slider_data("Param_MIN_HAIRPIN",minhairpin*2,maxhairpin*2)
		self.set_slider_data("Param_MAX_HAIRPIN",minhairpin*2,maxhairpin*2)
		self.set_slider_data("Param_HAIRPIN_PAIRS",minhairpinpairs,maxhairpinpairs)
		self.set_slider_data("Param_MAX_LENGTH",minmaxlength,maxmaxlength)
		
	def get_slider_value(self,slidername):
		slider = self._widget.findChild(QSlider,slidername)
		return slider.value()

	def set_slider_value(self,slidername,sliderval):
		slider = self._widget.findChild(QSlider,slidername)
		slider.setValue(sliderval)

	def set_slider_data(self,slidername,slidermin,slidermax):
		slider = self._widget.findChild(QSlider,slidername)
		slider.setMinimum(slidermin)
		slider.setMaximum(slidermax)
		

	def experimental_button_pressed(self):
		self._widget.findChild(QPushButton,"GenerateButton").setVisible(True)
		

	def generator_button_pressed(self):
		self.tell_launchella("Generating Track...")

		isLaxGenerator = self._widget.findChild(QCheckBox,"LaxCheckBox").isChecked()
		

		xys,twidth,theight = Generator.generate([	self.MIN_STRAIGHT,self.MAX_STRAIGHT,
								self.MIN_CTURN,self.MAX_CTURN,
								self.MIN_HAIRPIN/2,self.MAX_HAIRPIN/2,
								self.HAIRPIN_PAIRS,
								self.MAX_LENGTH,
								1 if isLaxGenerator else 0,
								1 if self._widget.findChild(QComboBox,"WhichPreset").currentText()=="Bezier" else 0
								])

		im = Converter.convert("xys","png",(xys,twidth,theight))
		rospy.logerr("Mahayana Buddhism")

		#If full stack selected, convert into csv and launch as well
		tgFullStack = self._widget.findChild(QCheckBox,"FullStackTrackGenButton")
		if tgFullStack.isChecked():
			imgpath = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/rand.png')
			Converter.convert("png","ALL",imgpath,params=[self.get_noise_level()])

		self.tell_launchella("Track Gen Complete!")

		im.show()

		self.load_track_and_images()

	def track_from_image_button_pressed(self):
		self.tell_launchella("Preparing to launch image as a track... ")
		fname = self._widget.findChild(QComboBox,"WhichImage").currentText()
		fname_full = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+fname)
		imFullStack = self._widget.findChild(QCheckBox,"FullStackImageButton")
		if imFullStack.isChecked():
			Converter.convert("png","ALL",fname_full,params=[self.get_noise_level()])
		else:
			Converter.convert("png","launch",fname_full,params=[self.get_noise_level()])

		self.launchfileoverride = fname[:-4] + ".launch"
		self.load_track_and_images()
		self.launch_button_pressed()

	

	def get_noise_level(self):
		noiseLevelWidget = self._widget.findChild(QSlider,"Noisiness")
		return (1.0*(noiseLevelWidget.value()-noiseLevelWidget.minimum()))/(noiseLevelWidget.maximum()-noiseLevelWidget.minimum())

	def convert_button_pressed(self):
		fromtype = self._widget.findChild(QComboBox,"ConvertFrom").currentText()
		totype   = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		filename = self._widget.findChild(QComboBox,"FileForConversion").currentText()
		self.tell_launchella("Conversion Button Pressed!  From: " + fromtype + " To: " + totype + " For: " + filename)
		midpointWidget = self._widget.findChild(QCheckBox,"MidpointBox")
		if fromtype == "png":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+filename)
		elif fromtype == "launch":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/'+filename)
		elif fromtype == "csv":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks/'+filename)
		suffix = "_CT" if self._widget.findChild(QCheckBox,"SuffixBox").isChecked() else ""
		Converter.convert(fromtype,totype,filename,
					params=[self.get_noise_level(),midpointWidget.isVisible() and midpointWidget.isChecked()],conversion_suffix=suffix)
		self.load_track_and_images()
		self.tell_launchella("Conversion Succeeded!  From: " + fromtype + " To: " + totype + " For: " + filename)

	def launch_button_pressed(self):
		if self.hasLaunchedROS:
			#Don't let people press launch twice
			#There's not really any reason why not to, but it's "undefined behavior"
			return
		self.hasLaunchedROS = True

		self.tell_launchella("--------------------------")
		self.tell_launchella("\t\t\tLaunching Nodes...")
		trackToLaunch = self.launchfileoverride #if we have set a specific file to run regardless of selected file
		self.launchfileoverride = None          #such as when we use the random generator
		if not trackToLaunch:
			trackToLaunch = self._widget.findChild(QComboBox,"WhichTrack").currentText()
		self.tell_launchella("Launching " + trackToLaunch)
		noiseLevel = self.get_noise_level()
		self.tell_launchella("With Noise Level: " + str(noiseLevel))
		
		controlMethod = "controlMethod:=speed"
		if self._widget.findChild(QRadioButton,"SpeedRadio").isChecked():
			self.tell_launchella("With Speed Controls")
			controlMethod = "controlMethod:=speed"
		elif self._widget.findChild(QRadioButton,"TorqueRadio").isChecked():
			self.tell_launchella("With Torque Controls")
			controlMethod = "controlMethod:=torque"
		
		#Ideally we would use the commented out 'launch' lines, but the ROSLaunchParent API does not allow sending arguments in Kinetic >:(
		#So we have to settle for launching this process using Python's Popen instead of the rospy API functions.
		if self.popenprocess:
			self.process.kill()
		self.popenprocess = self.launch_node_with_args(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch', trackToLaunch),[controlMethod])
		#launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch', trackToLaunch)])
		#launch.start()
		#self.launches.append(launch)


		if self._widget.findChild(QCheckBox,"VisualisatorCheckbox").isChecked():
			self.tell_launchella("And With LIDAR Data Visualisator.")
			self.launch_node(os.path.join(rospkg.RosPack().get_path('eufs_description'), "launch/visualisator.launch"))
		self.tell_launchella("As I have fulfilled my purpose in guiding you to launch a track, this launcher will no longer react to input.")

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
		# uncomment to save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# uncomment to restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	def launch_node(self,filepath):
		self.launch_node_with_args(filepath,[])

	def launch_node_with_args(self,filepath,args):
		if len(args) > 0:#We cannot use ROS' api with arguments in Kinetic
			process = Popen(["roslaunch",filepath]+args)
			self.popens.append(process)
			return process
		else:
			launch = roslaunch.parent.ROSLaunchParent(self.uuid, filepath)
			launch.start()
			self.launches.append(launch)
			return launch

	def shutdown_plugin(self):
		# unregister all publishers, kill all nodes
		self.tell_launchella("Shutdown Engaged...")
		#(Stop all processes)
		for p in self.processes:
			p.stop()

		for l in self.launches:
			l.shutdown()

		for p in self.popens:
			p.kill()

		#Trying to kill here will make a horrible bug
		#no idea why.  You'll want: "killall -9 gzserver gzclient" to fix it
		#if self.popenprocess != None:
		#	self.popenprocess.kill()
		#	self.popenprocess = None

		#NUKE IT! (seriously just nuke it)
		#self.nuke_ros()

		extranodes = rosnode.get_node_names()
		extranodes.remove("/eufs_launcher")
		extranodes.remove("/rosout")
		if (len(extranodes)>0):
			rospy.logerr("Warning, after shutting down the launcher, these nodes are still running: " + str(extranodes))

		nodes_to_kill = [	"/cone_ground_truth",
					"/eufs/controller_spawner",
					"/gazebo",
					"/gazebo_gui",
					"/robot_state_publisher",
					"/ros_can_sim",
					"/twist_to_ackermannDrive",
					"/spawn_platform",
					"/eufs_sim_rqt",
				]
		for badnode in extranodes:
			if badnode in nodes_to_kill:
				Popen(["rosnode","kill",badnode])
		time.sleep(1)
		extranodes = rosnode.get_node_names()
		extranodes.remove("/eufs_launcher")
		extranodes.remove("/rosout")
		if len(extranodes)>0:
			rospy.logerr("Pruned to: " + str(extranodes))

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog


















