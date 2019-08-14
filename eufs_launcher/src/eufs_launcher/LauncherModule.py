import os
import rospy
import rospkg
import roslaunch
import rosnode
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox

from os import listdir
from os.path import isfile, join

from subprocess import Popen

from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform

from TrackGenerator import TrackGenerator as Generator

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

		# Define colors for track gen and image reading
		self.noisecolor = (0,255,255,255)	#cyan, the turquoise turqouise wishes it were
		self.bgcolor = (255,255,255,255)	#white
		self.conecolor  = (255,0,255,255)	#magenta, for yellow cones
		self.conecolor2 = (0,0,255,255)		#blue, for blue cones
		self.carcolor = (0,255,0,255)		#green
		self.trackcenter = (0,0,0,255)		#black
		self.trackinner = (255,0,0,255)		#red
		self.trackouter = (255,255,0,255)	#yellow

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

		print("Plugin Successfully Launched!")

	def experimental_button_pressed(self):
		self._widget.findChild(QPushButton,"GenerateButton").setVisible(True)
		

	def generator_button_pressed(self):
		GENERATED_FILENAME = "rand"
		print("Generating Track...")

		xys,twidth,theight = Generator.generate("Contest Rules")

		#Create image to hold data
		im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0)) 
		draw = ImageDraw.Draw(im)

		#Convert data to image format
		draw.polygon([(0,0),(twidth,0),(twidth,theight),(0,theight),(0,0)],fill='white')#background
		draw.line(xys,fill=self.trackouter,width=5)#track full width
		draw.line(xys,fill=self.trackinner,width=3)#track innards
		draw.line(xys,fill=self.trackcenter)#track center


		pixels = im.load()#get reference to pixel data

		"""
		prevcone1 = None
		for index in range(1,len(xys)):
			#We will calculate normal vectors and put cones there
			#Line below calculates the normal
			(dx,dy) = normalizevec((1,endtangent([xys[index-1],xys[index]])+math.pi))
			(x,y) = xys[index]
			scalefactor = 7
			(ntx,nty) = (int(x+scalefactor*dx),int(y+scalefactor*dy))
			(nbx,nby) = (int(x-scalefactor*dx),int(y-scalefactor*dy))
			if prevcone1 == None:
				pixels[ntx,nty] = self.conecolor2
				pixels[nbx,nby] = self.conecolor
				prevcone1 = (nbx,nby)
			else:
				#Make sure the same colored cones go on the same side!
				(pcx,pcy) = prevcone1
				topcloseness = abs(ntx-pcx)+abs(nty-pcy)
				bottomcloseness = abs(nbx-pcx)+abs(nby-pcy)
				if bottomcloseness<topcloseness:
					pixels[ntx,nty] = self.conecolor2
					pixels[nbx,nby] = self.conecolor
					prevcone1 = (nbx,nby)
				else:
					pixels[ntx,nty] = self.conecolor
					pixels[nbx,nby] = self.conecolor2
					prevcone1 = (ntx,nty)
		"""	

		#We want to calculate direction of car position -
		sx = xys[0][0]
		sy = xys[0][1]
		ex = xys[1][0]
		ey = xys[1][1]
		angle = math.atan2(ey-sy,ex-sx)#[-pi,pi] but we want [0,2pi]
		if angle < 0:
			angle+=2*math.pi
		pixelValue = int(angle/(2*math.pi)*254+1) # it is *254+1 because we never want it to be 0
		if pixelValue > 255: pixelValue = 255
		if pixelValue <   1: pixelValue =   1
		colorforcar = (self.carcolor[0],self.carcolor[1],self.carcolor[2],pixelValue)

		draw.line([xys[0],xys[0]],fill=colorforcar)#car position

		

		#Now we want to make all pixels boardering the track become magenta (255,0,255) - this will be our 'cone' color
		#To find pixel boardering track, simply find white pixel adjacent to a non-white non-magenta pixle
		#We will also make sure not to put cones next to eachother	

		pixels = im.load()#get reference to pixel data

		def istrack(c):
			return c == self.trackouter or c == self.trackinner or c == self.trackcenter

		for i in range(im.size[0]):
			for j in range(im.size[1]):
				if pixels[i,j] == self.bgcolor:
					isNextToTrack = False
					isNextToCone = False
					if i>0:
						p=pixels[i-1,j]
						isNextToTrack = isNextToTrack or istrack(p)
						isNextToCone  = isNextToCone  or p==self.conecolor
					if j>0:
						p=pixels[i,j-1]
						isNextToTrack = isNextToTrack or istrack(p)
						isNextToCone  = isNextToCone  or p==self.conecolor
					if i<twidth-1:
						p=pixels[i+1,j]
						isNextToTrack = isNextToTrack or istrack(p)
						isNextToCone  = isNextToCone  or p==self.conecolor
					if j<theight-1:
						p=pixels[i,j+1]
						isNextToTrack = isNextToTrack or istrack(p)
						isNextToCone  = isNextToCone  or p==self.conecolor
					if isNextToTrack and not isNextToCone:
						pixels[i,j] = self.conecolor

		
		#We want to make the track have differing cone colors - yellow on inside, blue on outside
		#All cones are currently yellow.  We'll do a "breadth-first-search" for any cones reachable
		#from (0,0) and call those the 'outside' [(0,0) always blank as image is padded)]
		def getAllowedAdjacents(explolist,curpix):
			(i,j) = curpix
			allowed = []
			if i > 0:
				if not( (i-1,j) in explolist ):
					allowed.append((i-1,j))
			if j > 0:
				if not( (i,j-1) in explolist ):
					allowed.append((i,j-1))
			if i < twidth-1:
				if not( (i+1,j) in explolist ):
					allowed.append((i+1,j))
			if j < theight-1:
				if not( (i,j+1) in explolist ):
					allowed.append((i,j+1))
			return allowed
		print("Coloring cones...")
		exploredlist = set([])
		frontier = set([(0,0)])
		while len(frontier)>0:
			newfrontier = set([])
			for f in frontier:
				(i,j) = f
				pix = pixels[i,j]
				if pix == self.conecolor:
					pixels[i,j] = self.conecolor2
					newfrontier.update(getAllowedAdjacents(exploredlist,(i,j)))
				elif pix == self.bgcolor:
					newfrontier.update(getAllowedAdjacents(exploredlist,(i,j)))
				exploredlist.add(f)
			frontier = newfrontier
			#curexplored = len(exploredlist)
			#maxexplored = twidth*theight*1.0
			#print("Max Percent: " + str(curexplored/maxexplored))
				
		

		#Finally, we just need to place noise.  At maximal noise, the track should be maybe 1% covered? (that's actually quite a lot!)

		for i in range(im.size[0]):
			for j in range(im.size[1]):
				if pixels[i,j] == self.bgcolor:
					if uniform(0,100) < 1:#1% covered maximal noise
						pixels[i,j] = self.noisecolor

		im.save(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+GENERATED_FILENAME+'.png'))
		print("Image Generation Done, Converting to .world...")

		#self.create_track_from_image(im,GENERATED_FILENAME)

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


















