import os
import rospy
import rospkg
import roslaunch

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox

from os import listdir
from os.path import isfile, join

from subprocess import Popen

from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform

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
			if f != "rand.png":
				self._widget.findChild(QComboBox,"WhichImage").addItem(f)

		# Hook up buttons to onclick functions
		self._widget.findChild(QPushButton,"LaunchButton").clicked.connect(self.launch_button_pressed)
		self._widget.findChild(QPushButton,"GenerateButton").clicked.connect(self.generator_button_pressed)
		self._widget.findChild(QPushButton,"LoadFromImageButton").clicked.connect(self.track_from_image_button_pressed)

		#Create array of running processes (currently unused, but if you ever do process = launch.launch(node), add process here!)
		self.processes = []
		#And also an array of running launches
		self.launches = []

		# Add widget to the user interface
	        context.add_widget(self._widget)

		# Define colors for track gen and image reading
		self.noisecolor = (0,255,255,255)	#cyan, the turquoise turqouise wishes it were
		self.bgcolor = (255,255,255,255)	#white
		self.conecolor = (255,0,255,255)	#magenta
		self.carcolor = (0,255,0,255)		#green
		self.trackcenter = (0,0,0,255)		#black
		self.trackinner = (255,0,0,255)		#red
		self.trackouter = (255,255,0,255)	#yellow

		print("Plugin Successfully Launched!")
		self.hasLaunchedROS = False
		self.launchfileoverride = None


	def generator_button_pressed(self):
		GENERATED_FILENAME = "rand"
		print("Generating Track...")
		twidth = 100
		theight = 100
		im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0)) 
		draw = ImageDraw.Draw(im)
		ts = [t/100.0 for t in range(101)]

		#(All comments assume twidth = theight = 100 because I wrote them before I parameterized it)
		#Let us have a fixed start at (100,50) and end at (0,50) [we'll draw curve flipped for ease of maths]
		#and generate random control points
		#First choose a number of control points "nc" from 5 to 15
		#At (100/nc)*c for all control points c, choose random y from 0 to 100
		nc = randrange(5,15)
		xys = []
		closedloop = randrange(0,10)>=5

		if not closedloop:
			#Just generate a random bezier curve
			for c in range(0,nc+1):
				xys = xys + [(5+c*(twidth-10)/nc,uniform(5,theight-10))]
		else:
			#Generate a bezier that doubles back on itself
			for c in range(0,nc+1):
				xys = xys + [(5+c*(twidth-10)/nc,uniform(5,theight/2))]
			for c in range(0,nc):
				xys = xys + [(5+(nc-c)*(twidth-10)/nc,uniform(theight/2,theight-10))] + [xys[0]]

		bezier = make_bezier(xys)
		points = bezier(ts)

		#xys = [(end, point), (other randomly, generated points), (start, point)] to close the track into a loop!
		#bezier = make_bezier(xys) #Probably better if done like a triangle with 3 points instead.
		#points.extend(bezier(ts))

		draw.polygon([(0,0),(twidth,0),(twidth,theight),(0,theight),(0,0)],fill='white')#background
		draw.line(points,fill=self.trackouter,width=5)#track full width
		draw.line(points,fill=self.trackinner,width=3)#track innards
		draw.line(points,fill=self.trackcenter)#track center
		draw.line([xys[0],xys[0]],fill=self.carcolor)#car position

		#Now we want to make all pixels boardering the track become magenta (255,0,255) - this will be our 'cone' color
		#To find pixel boardering track, simply find white pixel adjacent to a non-white non-magenta pixle
		#If this is too thick, just destroy every second cone (or every third, or every fourth...  something like that!)
		#(For efficiency's sake we're actually looking for non-white non-magenta pixels, and making every white adjacent become magenta.	


		pixels = im.load()#get reference to pixel data
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				p = pixels[i,j]
				if p != self.bgcolor and p != self.conecolor:
					if i!=0 and j!=0:
						if pixels[i-1,j-1] == self.bgcolor:
							pixels[i-1,j-1] = self.conecolor
					if i!=twidth-1 and j!=0:
						if pixels[i+1,j-1] == self.bgcolor:
							pixels[i+1,j-1] = self.conecolor
					if i!=twidth-1 and j!=theight-1:
						if pixels[i+1,j+1] == self.bgcolor:
							pixels[i+1,j+1] = self.conecolor
					if i!=0 and j!=theight-1:
						if pixels[i-1,j+1] == self.bgcolor:
							pixels[i-1,j+1] = self.conecolor

		#Finally, we just need to place noise.  At maximal noise, the track should be maybe 10% covered? (that's actually quite a lot!)

		for i in range(im.size[0]):
			for j in range(im.size[1]):
				if pixels[i,j] == self.bgcolor:
					if uniform(0,100) < 1:#1% covered maximal noise
						pixels[i,j] = self.noisecolor

		im.save(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+GENERATED_FILENAME+'.png'))
		print("Image Generation Done, Converting to .world...")

		self.create_track_from_image(im,GENERATED_FILENAME)

		print("Track Gen Complete!")

		self.launchfileoverride = GENERATED_FILENAME + ".launch"
		self.launch_button_pressed()

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
		#params = %FILLNAME% %PLACEX% %PLACEY%
		launch_merged = "".join(launch_template)
		launch_merged = GENERATED_FILENAME.join(launch_merged.split("%FILLNAME%"))

		def xCoordTransform(x):
			return x-50
		def yCoordTransform(y):
			return y-50

		#Get PLACEX,PLACEY (look for (0,255,0,255))
		pixels = im.load()
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				if pixels[i,j] == self.carcolor:
					launch_merged = str(xCoordTransform(i)).join(launch_merged.split("%PLACEX%"))
					launch_merged = str(yCoordTransform(j)).join(launch_merged.split("%PLACEY%"))

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
		#ModelParams = %PLACEX% %PLACEY% %MODELNAME% %FILLCOLLISION%
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
		print(noisefiles)
		noiseweightings  = [(float(line[0]),line[1]) for line in noiseweightings_]

		def getRandomNoiseModel():
			randval = uniform(0,100)
			for a in noiseweightings:
				if a[0]>randval:
					return a[1]
			return "model://eufs_description/meshes/NoiseCube.dae"
		
		#Let's place all the models!
		def putModelAtPosition(mod,x,y):
			return str(xCoordTransform(x)).join(str(yCoordTransform(y)).join(mod.split("%PLACEY%")).split("%PLACEX%"))

		sdf_allmodels = ""
		noiseLevel = self.getNoiseLevel()
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				p = pixels[i,j]
				if p == self.conecolor:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_blueconemodel,i,j)
				elif p == self.noisecolor and uniform(0,1)<noiseLevel:
					sdf_noisemodel = getRandomNoiseModel().join(sdf_model.split("%MODELNAME%"))
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
		p = Popen(["roslaunch",os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'launch', trackToLaunch),controlMethod])
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
		print("Shutdown Complete!")


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

# Bezier code taken from https://stackoverflow.com/questions/246525/how-can-i-draw-a-bezier-curve-using-pythons-pil

def make_bezier(xys):
	# xys should be a sequence of 2-tuples (Bezier control points)
	n = len(xys)
	combinations = pascal_row(n-1)
	def bezier(ts):
		# This uses the generalized formula for bezier curves
		# http://en.wikipedia.org/wiki/B%C3%A9zier_curve#Generalization
		result = []
		for t in ts:
			tpowers = (t**i for i in range(n))
			upowers = reversed([(1-t)**i for i in range(n)])
			coefs = [c*a*b for c, a, b in zip(combinations, tpowers, upowers)]
			result.append(
				tuple(sum([coef*p for coef, p in zip(coefs, ps)]) for ps in zip(*xys)))
		return result
	return bezier

def pascal_row(n):
	# This returns the nth row of Pascal's Triangle
	result = [1]
	x, numerator = 1, n
	for denominator in range(1, n//2+1):
		# print(numerator,denominator,x)
		x *= numerator
		x /= denominator
		result.append(x)
		numerator -= 1
	if n&1 == 0:
		# n is even
		result.extend(reversed(result[:-1]))
	else:
		result.extend(reversed(result)) 
	return result
