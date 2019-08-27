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
from TrackGenerator import GenerationFailedException

from ConversionTools import ConversionTools as Converter

class EufsLauncher(Plugin):

	def __init__(self, context):
		"""
		This function handles loading the launcher GUI and all the setting-up of the values and buttons displayed.
		"""
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
		preset_names = Generator.get_preset_names()

		#Add Presets to Preset Selector (always put Computer Friendly first)
		default_preset = Generator.get_default_preset()
		if default_preset in preset_names:
			self._widget.findChild(QComboBox,"WhichPreset").addItem(default_preset)
		for f in preset_names:
			if f != default_preset:
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
		self.has_launched_ros = False
		self.launch_file_override = None
		self.popen_process = None

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
		self.ignore_slider_changes = False

		#Setup Lax Generation button
		lax_botton = self._widget.findChild(QCheckBox,"LaxCheckBox")
		lax_botton.setChecked(self.LAX_GENERATION)
		
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
		midpoint_box = self._widget.findChild(QCheckBox,"MidpointBox")
		convert_to = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		#midpoint_box.setVisible(convert_to=="csv" or convert_to=="ALL")
		midpoint_box.setChecked(True)

		#Suffix checkbox
		suffix_box = self._widget.findChild(QCheckBox,"SuffixBox")
		suffix_box.setChecked(True)

		#Track Gen full stack checkbox
		track_generator_full_stack = self._widget.findChild(QCheckBox,"FullStackTrackGenButton")
		track_generator_full_stack.setChecked(True)

		#Image full stack checkbox
		image_launcher_full_stack = self._widget.findChild(QCheckBox,"FullStackImageButton")
		image_launcher_full_stack.setChecked(True)

		#Copier full stack checkbox
		copier_full_stack = self._widget.findChild(QCheckBox,"FullStackCopyButton")
		copier_full_stack.setChecked(True)

		#Change label to show current selected file for the copier
		self.update_copier()

		#Get uuid
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)

	def tell_launchella(self,what):
		"""Display text in feedback box (lower left corner)."""
		self._widget.findChild(QLabel,"UserFeedbackLabel").setText(what)
		QApplication.processEvents() 

	def sketcher_button_pressed(self):
		"""Called when sketcher button is pressed, currently not in use."""
		loadUi(self.sketcher_ui_file, self._widget)

	def load_track_and_images(self):
		"""
		Peruses file system for files to add to the drop-down menus of the launcher.
		"""
		# Clear the dropdowns
		self._widget.findChild(QComboBox,"WhichTrack").clear()
		self._widget.findChild(QComboBox,"WhichImage").clear()
		# Get tracks from eufs_gazebo package
		relevant_path = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch')
		launch_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f))]

		#Remove "blacklisted" files (ones that don't define tracks)
		blacklist_ = open(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/blacklist.txt'),"r")
		blacklist = [f.strip() for f in blacklist_]#remove \n
		launch_files = [f for f in launch_files if not f in blacklist]

		# Add Tracks to Track Selector
		if "small_track.launch" in launch_files:
			self._widget.findChild(QComboBox,"WhichTrack").addItem("small_track.launch")
		for f in launch_files:
			if f != "small_track.launch":
				self._widget.findChild(QComboBox,"WhichTrack").addItem(f)

		# Get images
		relevant_path = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs')
		image_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f))]

		# Add Images to Image Selector (always put rand.png first)
		if "rand.png" in image_files:
			self._widget.findChild(QComboBox,"WhichImage").addItem("rand.png")
		for f in image_files:
			if f != "rand.png" and f[-3:] == "png":
				self._widget.findChild(QComboBox,"WhichImage").addItem(f)

	def copy_button_pressed(self):
		"""When copy button is pressed, launch ConversionTools"""
		self.tell_launchella("Copying...")
		#Copy the current file
		is_full_stack = self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked()
		file_to_copy_to = self._widget.findChild(QLineEdit,"RenameFileTextbox").text()
		file_to_copy_from = self._widget.findChild(QComboBox,"FileForConversion").currentText()
		raw_name_to = file_to_copy_to.split(".")[0]
		raw_name_from = file_to_copy_from.split(".")[0]
		ending = file_to_copy_from.split(".")[-1]

		if len(file_to_copy_to) == 0:#Don't let them create null-named files
			return

		if ending == "launch":
			#For launch files, we also need to move around the model folders
			if not os.path.exists(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',raw_name_to)):
				os.mkdir(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',raw_name_to))
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',raw_name_from,"model.sdf")
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',raw_name_to,"model.sdf")
			Converter.copy_file(path_from,path_to)

			path_from = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',raw_name_from,"model.config")
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',raw_name_to,"model.config")
			Converter.copy_file(path_from,path_to)
			
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch',file_to_copy_from)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'launch',raw_name_to+"."+ending)
			Converter.copy_file(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("launch","ALL",path_to,params=[self.get_noise_level()])

		elif ending == "png":
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs',file_to_copy_from)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'randgen_imgs',raw_name_to+"."+ending)
			Converter.copy_file(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("png","ALL",path_to,params=[self.get_noise_level()])

		elif ending == "csv":
			path_from = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks',file_to_copy_from)
			path_to   = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'tracks',raw_name_to+"."+ending)
			Converter.copy_file(path_from,path_to)

			#If full stack copying, convert to all file formats
			if self._widget.findChild(QCheckBox,"FullStackCopyButton").isChecked():
				Converter.convert("csv","ALL",path_to,params=[self.get_noise_level()])
		self.load_track_and_images()
		self.tell_launchella("Copy Succeeded!")

	def update_copier(self):
		"""Change label to show current selected file for the copier"""
		copy_head = self._widget.findChild(QLabel,"RenameFileHeader")
		copy_head.setText("Copy: "+self._widget.findChild(QComboBox,"FileForConversion").currentText())

	def update_midpoints_box(self):
		"""Controls the handling of the box that, when ticked, tells the to-csv converter to calculate cone midpoints."""
		#Toggle checkbox
		convert_to = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		#self._widget.findChild(QCheckBox,"MidpointBox").setVisible(convert_to=="csv" or convert_to=="ALL")

	def update_converter_dropdown(self):
		"""Keep the drop-down menus of ConversionTools in sync with the filesystem."""
		from_type = self._widget.findChild(QComboBox,"ConvertFrom").currentText()
		all_files = []

		if from_type == "launch":
			# Get tracks from eufs_gazebo package
			relevant_path = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch')
			launch_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f))]
	
			#Remove "blacklisted" files (ones that don't define tracks)
			blacklist_ = open(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/blacklist.txt'),"r")
			blacklist = [f.strip() for f in blacklist_]#remove \n
			all_files = [f for f in launch_files if not f in blacklist]
		elif from_type == "png":
			# Get images
			relevant_path = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs')
			all_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f))]
		elif from_type == "csv":
			#Get csvs
			relevant_path = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks')
			all_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f)) and f[-3:]=="csv"]

		#Remove old files from selector
		the_selector = self._widget.findChild(QComboBox,"FileForConversion")
		the_selector.clear()

		# Add files to selector
		for f in all_files:
			the_selector.addItem(f)

		self.update_copier()

	def update_preset(self):
		"""When preset is changed, change the sliders accordingly."""
		which = self._widget.findChild(QComboBox,"WhichPreset").currentText()
		preset_data = Generator.get_preset(which)
		self.MIN_STRAIGHT   = preset_data[0]
		self.MAX_STRAIGHT   = preset_data[1]
		self.MIN_CTURN      = preset_data[2]
		self.MAX_CTURN      = preset_data[3]
		self.MIN_HAIRPIN    = preset_data[4]*2
		self.MAX_HAIRPIN    = preset_data[5]*2
		self.HAIRPIN_PAIRS  = preset_data[6]
		self.MAX_LENGTH     = preset_data[7]
		self.LAX_GENERATION = preset_data[8]
		self.TRACK_WIDTH    = 4
		self._widget.findChild(QCheckBox,"LaxCheckBox").setChecked(self.LAX_GENERATION)

	def keep_track_of_preset_changes(self):
		"""Hooks up the preset button with the preset_changed function."""
		self._widget.findChild(QComboBox,"WhichPreset") .currentTextChanged.connect(self.preset_changed)

	def preset_changed(self):
		"""
		When preset is changed, set everything into motion that needs to happen.
		
		Updates dropdowns and sliders.
		"""
		self.ignore_slider_changes = True
		self.update_preset()
		self.keep_params_up_to_date()
		self.keep_sliders_up_to_date()
		self.ignore_slider_changes = False

	def keep_track_of_slider_changes(self):
		"""
		Hooks up all sliders with functions to respond to their changes.
		"""
		self._widget.findChild(QSlider,"Param_MIN_STRAIGHT") .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_STRAIGHT") .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MIN_CTURN")    .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_CTURN")    .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MIN_HAIRPIN")  .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_HAIRPIN")  .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_HAIRPIN_PAIRS").valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_MAX_LENGTH")   .valueChanged.connect(self.slider_changed)
		self._widget.findChild(QSlider,"Param_TRACK_WIDTH")  .valueChanged.connect(self.slider_changed)
		
	def slider_changed(self):
		"""When a slider is changed, update the parameters."""
		if self.ignore_slider_changes: return
		self.keep_variables_up_to_date()
		self.keep_params_up_to_date()

	def keep_params_up_to_date(self):
		"""This function keeps the labels next to the sliders up to date with the actual values."""
		self._widget.findChild(QLabel,"Label_MIN_STRAIGHT") .setText("MIN_STRAIGHT: "  + str(self.MIN_STRAIGHT))
		self._widget.findChild(QLabel,"Label_MAX_STRAIGHT") .setText("MAX_STRAIGHT: "  + str(self.MAX_STRAIGHT))
		self._widget.findChild(QLabel,"Label_MIN_CTURN")    .setText("MIN_CTURN: "     + str(self.MIN_CTURN))
		self._widget.findChild(QLabel,"Label_MAX_CTURN")    .setText("MAX_CTURN: "     + str(self.MAX_CTURN))
		self._widget.findChild(QLabel,"Label_MIN_HAIRPIN")  .setText("MIN_HAIRPIN: "   + str((self.MIN_HAIRPIN/2.0)))
		self._widget.findChild(QLabel,"Label_MAX_HAIRPIN")  .setText("MAX_HAIRPIN: "   + str((self.MAX_HAIRPIN/2.0)))
		self._widget.findChild(QLabel,"Label_HAIRPIN_PAIRS").setText("HAIRPIN_PAIRS: " + str(self.HAIRPIN_PAIRS))
		self._widget.findChild(QLabel,"Label_MAX_LENGTH")   .setText("MAX_LENGTH: "    + str(self.MAX_LENGTH))
		self._widget.findChild(QLabel,"Label_TRACK_WIDTH")  .setText("TRACK_WIDTH: "  + str(self.TRACK_WIDTH	))

	def keep_sliders_up_to_date(self):
		"""This function keeps the values of the sliders up to date with the actual values."""
		self.set_slider_value("Param_MIN_STRAIGHT",self.MIN_STRAIGHT)
		self.set_slider_value("Param_MAX_STRAIGHT",self.MAX_STRAIGHT)
		self.set_slider_value("Param_MIN_CTURN",self.MIN_CTURN)
		self.set_slider_value("Param_MAX_CTURN",self.MAX_CTURN)
		self.set_slider_value("Param_MIN_HAIRPIN",self.MIN_HAIRPIN)
		self.set_slider_value("Param_MAX_HAIRPIN",self.MAX_HAIRPIN)
		self.set_slider_value("Param_HAIRPIN_PAIRS",self.HAIRPIN_PAIRS)
		self.set_slider_value("Param_MAX_LENGTH",self.MAX_LENGTH)
		self.set_slider_value("Param_TRACK_WIDTH",self.TRACK_WIDTH)

	def keep_variables_up_to_date(self):
		"""This function keeps LauncherModule's variables in tune with the slider values."""
		self.MIN_STRAIGHT = self.get_slider_value("Param_MIN_STRAIGHT")
		self.MAX_STRAIGHT = self.get_slider_value("Param_MAX_STRAIGHT")
		self.MIN_CTURN = self.get_slider_value("Param_MIN_CTURN")
		self.MAX_CTURN = self.get_slider_value("Param_MAX_CTURN")
		self.MIN_HAIRPIN = self.get_slider_value("Param_MIN_HAIRPIN")
		self.MAX_HAIRPIN = self.get_slider_value("Param_MAX_HAIRPIN")
		self.HAIRPIN_PAIRS = self.get_slider_value("Param_HAIRPIN_PAIRS")
		self.MAX_LENGTH = self.get_slider_value("Param_MAX_LENGTH")
		self.TRACK_WIDTH = self.get_slider_value("Param_TRACK_WIDTH")

	def set_slider_ranges(self):
		"""
		This function specifies the bounds of the sliders.

		Note on ranges: 
				Straights are between 0 and 150
				Turns are between 0 and 50
				Hairpins are between 0 and 20, and have half-step rather than integer step (hence scaling by 2s)
				Hairpin pairs are between 0 and 5
		               Max Length is between 200 and 2000
		"""
		min_straight = 0
		max_straight = 150
		min_turn = 0
		max_turn = 50
		min_hairpin = 0
		max_hairpin = 20
		min_hairpin_pairs = 0
		max_hairpin_pairs = 5
		min_max_length = 200
		max_max_length = 2000
		min_width = 2
		max_width = 10
		self.set_slider_data("Param_MIN_STRAIGHT",min_straight,max_straight)
		self.set_slider_data("Param_MAX_STRAIGHT",min_straight,max_straight)
		self.set_slider_data("Param_MIN_CTURN",min_turn,max_turn)
		self.set_slider_data("Param_MAX_CTURN",min_turn,max_turn)
		self.set_slider_data("Param_MIN_HAIRPIN",min_hairpin*2,max_hairpin*2)
		self.set_slider_data("Param_MAX_HAIRPIN",min_hairpin*2,max_hairpin*2)
		self.set_slider_data("Param_HAIRPIN_PAIRS",min_hairpin_pairs,max_hairpin_pairs)
		self.set_slider_data("Param_MAX_LENGTH",min_max_length,max_max_length)
		self.set_slider_data("Param_TRACK_WIDTH",min_width,max_width)
		
	def get_slider_value(self,slidername):
		"""Returns the value of the specified slider."""
		slider = self._widget.findChild(QSlider,slidername)
		return slider.value()

	def set_slider_value(self,slidername,sliderval):
		"""Sets the value of the specified slider."""
		slider = self._widget.findChild(QSlider,slidername)
		slider.setValue(sliderval)

	def set_slider_data(self,slidername,slidermin,slidermax):
		"""Sets the minimum and maximum values of sliders."""
		slider = self._widget.findChild(QSlider,slidername)
		slider.setMinimum(slidermin)
		slider.setMaximum(slidermax)
		

	def experimental_button_pressed(self):
		"""When features are listed as experimental, then they are invisible until this function switches them on."""
		pass
		#Example: self._widget.findChild(QPushButton,"GenerateButton").setVisible(True)
		

	def generator_button_pressed(self):
		"""Handles random track generation by accessing TrackGenerator and ConversionTools."""
		self.tell_launchella("Generating Track...")

		isLaxGenerator = self._widget.findChild(QCheckBox,"LaxCheckBox").isChecked()
		
		try:

			xys,twidth,theight = Generator.generate([	self.MIN_STRAIGHT,self.MAX_STRAIGHT,
									self.MIN_CTURN,self.MAX_CTURN,
									self.MIN_HAIRPIN/2,self.MAX_HAIRPIN/2,
									self.HAIRPIN_PAIRS,
									self.MAX_LENGTH,
									1 if isLaxGenerator else 0,
									1 if self._widget.findChild(QComboBox,"WhichPreset").currentText()=="Bezier" else 0
									])
			self.tell_launchella("Loading Image...")
			im = Converter.convert("xys","png",(xys,twidth,theight),params=[self.TRACK_WIDTH])

			#If full stack selected, convert into csv and launch as well
			track_generator_full_stack = self._widget.findChild(QCheckBox,"FullStackTrackGenButton")
			if track_generator_full_stack.isChecked():
				img_path = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/rand.png')
				Converter.convert("png","ALL",img_path,params=[self.get_noise_level()])

			self.tell_launchella("Track Gen Complete!")

			im.show()
		except GenerationFailedException:
			rospy.logerr(	"\nError!  The generator could not generate a track in time.\n"+
					"Maybe try different parameters?\n"+
					"Turning on Lax Generation and increasing MAX_STRAIGHT and MIN_STRAIGHT usually helps.")
			self.tell_launchella("Track Gen Failed :(  Try different parameters?")

		self.load_track_and_images()

	def track_from_image_button_pressed(self):
		"""Converts .png to .launch by interfacing with ConversionTools, then launches said .launch."""
		self.tell_launchella("Preparing to launch image as a track... ")
		filename = self._widget.findChild(QComboBox,"WhichImage").currentText()
		filename_full = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+filename)
		image_launcher_full_stack = self._widget.findChild(QCheckBox,"FullStackImageButton")
		if image_launcher_full_stack.isChecked():
			Converter.convert("png","ALL",filename_full,params=[self.get_noise_level()])
		else:
			Converter.convert("png","launch",filename_full,params=[self.get_noise_level()])

		self.launch_file_override = filename[:-4] + ".launch"
		self.load_track_and_images()
		self.launch_button_pressed()

	

	def get_noise_level(self):
		"""Returns the noise slider's noise level."""
		noise_level_widget = self._widget.findChild(QSlider,"Noisiness")
		return (1.0*(noise_level_widget.value()-noise_level_widget.minimum()))/(noise_level_widget.maximum()-noise_level_widget.minimum())

	def convert_button_pressed(self):
		"""Handles interfacing with ConversionTools."""
		from_type = self._widget.findChild(QComboBox,"ConvertFrom").currentText()
		to_type   = self._widget.findChild(QComboBox,"ConvertTo").currentText()
		filename = self._widget.findChild(QComboBox,"FileForConversion").currentText()
		self.tell_launchella("Conversion Button Pressed!  From: " + from_type + " To: " + to_type + " For: " + filename)
		midpoint_widget = self._widget.findChild(QCheckBox,"MidpointBox")
		if from_type == "png":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+filename)
		elif from_type == "launch":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch/'+filename)
		elif from_type == "csv":
			filename = os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks/'+filename)
		suffix = "_CT" if self._widget.findChild(QCheckBox,"SuffixBox").isChecked() else ""
		Converter.convert(from_type,to_type,filename,
					params=[self.get_noise_level(),midpoint_widget.isVisible() and midpoint_widget.isChecked()],conversion_suffix=suffix)
		self.load_track_and_images()
		self.tell_launchella("Conversion Succeeded!  From: " + from_type + " To: " + to_type + " For: " + filename)

	def launch_button_pressed(self):
		"""Launches Gazebo."""
		if self.has_launched_ros:
			#Don't let people press launch twice
			#There's not really any reason why not to, but it's "undefined behavior"
			return
		self.has_launched_ros = True

		self.tell_launchella("--------------------------")
		self.tell_launchella("\t\t\tLaunching Nodes...")
		track_to_launch = self.launch_file_override #if we have set a specific file to run regardless of selected file
		self.launch_file_override = None          #such as when we use the random generator
		if not track_to_launch:
			track_to_launch = self._widget.findChild(QComboBox,"WhichTrack").currentText()
		self.tell_launchella("Launching " + track_to_launch)
		noise_level = self.get_noise_level()
		self.tell_launchella("With Noise Level: " + str(noise_level))
		
		control_method = "controlMethod:=speed"
		if self._widget.findChild(QRadioButton,"SpeedRadio").isChecked():
			self.tell_launchella("With Speed Controls")
			control_method = "controlMethod:=speed"
		elif self._widget.findChild(QRadioButton,"TorqueRadio").isChecked():
			self.tell_launchella("With Torque Controls")
			control_method = "controlMethod:=torque"

		perception_stack = ["launch_group:=no_perception"]
		if self._widget.findChild(QCheckBox,"PerceptionCheckbox").isChecked():
			perception_stack = []#is on
		

		if self.popen_process:
			self.process.kill()

		#How we launch the simulation changes depending on whether
		dir_to_check = os.path.dirname(os.path.dirname(os.path.dirname(rospkg.RosPack().get_path('eufs_gazebo'))))
		if dir_to_check.split("/")[-1] == "eufs-master":
			launch_location = os.path.join(dir_to_check, 
						'launch', 
						'simulation.launch')
			self.popen_process = 	self.launch_node_with_args(
						launch_location,
						[control_method,"track:="+track_to_launch.split(".")[0]]+perception_stack
					)
		else:
			self.popen_process = self.launch_node_with_args(
						os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch', track_to_launch),
						[control_method]
					)
			

		if self._widget.findChild(QCheckBox,"VisualisatorCheckbox").isChecked():
			self.tell_launchella("And With LIDAR Data Visualisator.")
			self.launch_node(os.path.join(rospkg.RosPack().get_path('eufs_description'), "launch","visualisator.launch"))
		self.tell_launchella("As I have fulfilled my purpose in guiding you to launch a track, this launcher will no longer react to input.")

	def nuke_ros(self):
		"""
		Kill everything about ROS - only used for debugging
		"""
		#Try to kill as much as possible
		#Burn it all to the ground
		Popen(["killall","-9","gzserver"])
		Popen(["killall","-9","gzclient"])
		extra_nodes= rosnode.get_node_names()
		extra_nodes.remove('/rosout')
		extra_nodes= [f for f in extra_nodes if f[:17] != '/rqt_gui_py_node_'] #remove the gui's node from kill list
		for bad_node in extra_nodes:
			Popen(["rosnode","kill",bad_node])


	def save_settings(self, plugin_settings, instance_settings):
		# uncomment to save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# uncomment to restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	def launch_node(self,filepath):
		"""Wrapper for launch_node_with_args"""
		self.launch_node_with_args(filepath,[])

	def launch_node_with_args(self,filepath,args):
		"""
		Launches ros node.

		If arguments are supplied, it has to use Popen rather than the default launch method.
		"""
		if len(args) > 0:#We cannot use ROS' api with arguments in Kinetic
			process = Popen(["roslaunch",filepath]+args)
			self.popens.append(process)
			return process
		else:
			launch = roslaunch.parent.ROSLaunchParent(self.uuid, [filepath])
			launch.start()
			self.launches.append(launch)
			return launch

	def shutdown_plugin(self):
		"""Unregister all publishers, kill all nodes."""
		self.tell_launchella("Shutdown Engaged...")
		#(Stop all processes)
		for p in self.processes:
			p.stop()

		for l in self.launches:
			l.shutdown()

		for p in self.popens:
			p.kill()

		#Manual node killer:
		
		extra_nodes= rosnode.get_node_names()
		extra_nodes.remove("/eufs_launcher")
		extra_nodes.remove("/rosout")
		left_open = len(extra_nodes)
		if (left_open>0):
			rospy.logerr("Warning, after shutting down the launcher, these nodes are still running: " + str(extra_nodes))

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
		for bad_node in extra_nodes:
			if bad_node in nodes_to_kill:
				Popen(["rosnode","kill",bad_node])
		Popen(["killall","-9","gzserver"])
		time.sleep(0.25)
		extra_nodes= rosnode.get_node_names()
		extra_nodes.remove("/eufs_launcher")
		extra_nodes.remove("/rosout")
		if left_open>0:
			rospy.logerr("Pruned to: " + str(extra_nodes))
		

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog


















