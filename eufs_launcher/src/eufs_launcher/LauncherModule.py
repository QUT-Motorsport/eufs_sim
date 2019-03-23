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

		#Generate the track as pure data
		xys = []
		overlapped = False
		while overlapped or xys==[]:
			#Re-generate if the track overlaps itself
			(xys,twidth,theight) = generateAutocrossTrackdriveTrack((0,0))
			xys = compactify_points(xys)
			overlapped = check_if_overlap(xys)
			if overlapped:
				print("Oops!  The track intersects itself too much.  Retrying...")

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

def endtangent(xys):
	#Calculate direction of outgoing tangent of a set of points
	#Is an angle!
	sx = xys[-2][0]
	sy = xys[-2][1]
	ex = xys[-1][0]
	ey = xys[-1][1]
	return math.atan2((ey-sy),(ex-sx))

def capangle(ang):
	#Returns angle between 0 and 2*math.pi
	if ang < 0:
		return capangle(ang+2*math.pi)
	elif ang >= 2*math.pi:
		return capangle(ang-2*math.pi)
	return ang

def capangle_odd(ang):
	#Returns angle between -math.pi and math.pi
	ang = capangle(ang)
	if ang > math.pi:
		ang = ang-2*math.pi
	return ang

def normalizevec(vec):
	(a,b) = vec
	mag = math.sqrt(a*a+b*b)
	return (a/mag,b/mag)

def generateAutocrossTrackdriveTrack(startpoint):
	#The rules:
	#	Straights: 		<=80 meters
	#	Constant Turns:		<=25 meter radius
	#	Hairpin Turns: 		>=4.5 meter outside radius
	#	Slalom: 		Cones in a straight line with 7.5 to 12 meter spacing [NOTE: can't be generated at this point, added later]
	#	Track Width: 		>=3 meters
	#	Track Length: 		<=1500 meters
	xys = []
	curTrackLength = 0
	curpoint = startpoint

	#Let's start with a small straght
	(generated, curpoint, deltalength) = generateStraight(startpoint,uniform(80,80),math.pi/8)
	curTrackLength += deltalength
	xys.extend(generated)

	#Now we want to set checkpoints to pass through:
	goalpoints = [(startpoint[0]+150,startpoint[1]),(startpoint[0]+200,startpoint[1]+150),(startpoint[0]-50,startpoint[1]+200)]
	for goalpoint in goalpoints:
		(generated, curpoint, length) = generatePathFromPointToPoint(curpoint,goalpoint,endtangent(xys),fuzzradius=20)
		curTrackLength+= length
		xys.extend(generated)

	#Now lets head back to the start:
	(generated, curpoint, length) = generatePathFromPointToPoint(curpoint,startpoint,endtangent(xys),fuzzradius=0)
	curTrackLength+= length
	xys.extend(generated)

	if curTrackLength > 1500:
		print("Track gen failed - track too long, oops!  Retrying.")
		return generateAutocrossTrackdriveTrack(startpoint)

	return convertPointsToAllPositive(xys)

def generatePathFromPointToPoint(startpoint,endpoint,intangent,depth=20,hairpined=False,manyhairpins=False,fuzzradius=0):
	#Here we want to get from a to b by randomly placing paths 
	#[Note: depth parameter is just to limit recursion overflows]
	#[And hairpined parameter prevents multiple hairpins - we should have at most
	#one or else its hard to generate nice paths]
	#[manyhairpins overrides this and allows an arbitrary amount]
	#[fuzzradius is how close to the end we want to be]
	length = 0
	points = []
	circleradius = uniform(25,25)


	"""
	#--------------------------------------------------
	#Here lies the circle code that did not take into account the fact that the circle itself changed where the "startpoint" was	
	#I'm keeping it here incase someone wants to modify it to take that into account.
	#I gave up and used an approximation method, creating the function generateConstantTurnUntilFacingPoint
	#--------------------------------------------------

	#First we want to direct ourselves towards endpoint, since intangent is the current direction
	goaltangent = endtangent([startpoint,endpoint])
	
	#We're going to place a cturn to point us in the right direction, but we need to calculate the percent needed
	#We can get the angles the lines are pointing at using atan
	#Draw a circle to see why, keep in mind tangent perpendicular to radius!
	#I've not simplified the numbers so its easier for you to re-derive
	inangle = math.pi-(math.pi/2-math.atan(intangent))
	goalangle = math.pi-(math.pi/2-math.atan(goaltangent))
	finalangle = goalangle-inangle
	finalpercent = finalangle/(2*math.pi)
	print("In: " + str(inangle))
	print("Goal: " + str(goalangle))
	#---------------------------------------------------
	"""

	#We want to know ahead of time which way the circle will turn - that way we don't get loopty-loops
	#where the circle turns nearly all the way around when it would have been better to have the center
	#on the other side.
	#We'll do this by calculating the normal we want.
	#First, we want to know the path from start to goal.
	#And more specifically, its angle
	directpathangle = endtangent([startpoint,endpoint])
	
	#Our circle code takes in "turnagainstnormal" - alternatively, this parameter is equivalent to passing in
	#a vector pointing TOWARDS the radius.  So let's calculate that!  If we have the goal be at a "higher" angle
	#than it, we want the radius to be up, otherwise down.  The angle of it pointing up is simply pi/2 higher than the tangent (90 degrees)
	normalangle = capangle(math.pi/2 + intangent)

	#We now calculate if we need to add an additional pi to it.
	#If directpathangle is higher than intangent - but how do we define higher?  We say that it is higher if the
	#counterclockwise angle difference is smaller than the clockwise difference.
	if capangle(directpathangle-normalangle)>capangle(directpathangle-directpathangle):
		normalangle = capangle(math.pi + normalangle)

	#Also flip it if tangent heading in 'negative' direction
	#if (abs(capangle_odd(intangent))math.pi/2):
	#	normalangle = capangle(math.pi + normalangle)

	#Finally lets convert this into a normal:
	thenormal = (1,math.tan(normalangle))
	
	#Now let's actually draw the circle!
	(generated, curpoint,deltalength,output_normal) = generateConstantTurnUntilFacingPoint(startpoint,circleradius,intangent,endpoint,turnagainstnormal = thenormal)
		#generateConstantTurn(startpoint,circleradius,intangent,circlepercent=finalpercent)
	length += deltalength
	points.extend(generated)

	#Check if we're within 80 of the goal:
	(cx,cy) = points[-1]
	(ex,ey) = endpoint
	squaredistance = (ex-cx)*(ex-cx)+(ey-cy)*(ey-cy)
	#print("--------------------------------")
	#print(math.sqrt(squaredistance))
	#print(points[-1])
	#print(endpoint)
	#print("++++++++++++++++++++++++++++++++")
	if squaredistance <= 80*80+fuzzradius*fuzzradius:
		#We'll just draw a straight to it
		#print("\n\nSUCCESS!!!\n\n")
		#print("Fuzz: " + str(squaredistance - 80*80))
		(generated, curpoint,deltalength) = generateStraight(points[-1],min(math.sqrt(squaredistance),80),endtangent(points))
		length += deltalength
		points.extend(generated)
	else:
		#Go as far as we can (unless we're very close in which case don't, because it'll cause the next iteration to look weird)
		straightsize = 80 if squaredistance <= 100*100 else 40
		(generated, curpoint,deltalength) = generateStraight(points[-1],straightsize,endtangent(points))
		length += deltalength
		points.extend(generated)
		#We'll either do a random cturn or a random hairpin, then continue the journey
		cturn_or_hairpin = uniform(0,1)
		makecturn = cturn_or_hairpin < 0.7
		if makecturn or (hairpined and not manyhairpins):#cturn
			(generated, curpoint,deltalength,output_normal) = generateConstantTurn(curpoint,uniform(10,25),endtangent(points),
											circlepercent=uniform(0,0.25),turnagainstnormal = output_normal)
			length += deltalength
			points.extend(generated)
			(generated, curpoint,deltalength,output_normal) = generateConstantTurn(curpoint,uniform(10,25),endtangent(points),
											circlepercent=uniform(0,0.25),turnagainstnormal = output_normal)
			length += deltalength
			points.extend(generated)
		else:
			#We only want an even amount of turns so that we can leave heading the same direction we entered.
			#otherwise due to the way we head towards the path, its basically guaranteed we get a self-intersection.
			numswitches = 2*randrange(1,3)
			(generated, curpoint,deltalength) = generateHairpinTurn(curpoint,uniform(4.5,10),endtangent(points),switchbacknum=numswitches)
			length += deltalength
			points.extend(generated)
		#Now we recurse!
		if depth>0:
			(generated, curpoint, length) = generatePathFromPointToPoint(curpoint,endpoint,endtangent(points),depth-1,
											hairpined=hairpined or not makecturn,manyhairpins=manyhairpins,
											fuzzradius=fuzzradius)
			length += deltalength
			points.extend(generated)
			


	return (points,points[-1],length)

def generateHairpinTurn(startpoint,radius,intangent,switchbacknum=None,turnleft=None,wobbliness=None,straightsize=None,circlesize=None,uniformcircles=True):
	curpoint = startpoint
	curtangent = intangent
	length = 0
	startleftnum = 0 if turnleft else 1
	
	#A hairpin has a few choices:
	#	How many switchbacks
	#	Direction of first switchback
	#	"Wobbliness" (circlepercent)
	#	Size of straightways
	turnleft = uniform(0,1)<0.5 		if turnleft == None 		else turnleft
	switchbacknum = randrange(2,10)		if switchbacknum == None	else switchbacknum
	wobbliness = uniform(0.45,0.55)		if wobbliness == None		else wobbliness
	straightsize = uniform(10,80)		if straightsize == None		else straightsize
	circlesize = uniform(10,25)		if circlesize == None		else circlesize

	#We are interested in making sure switchbacks never intersect
	#If you draw a diagram, this gives you a diamond with angles pi/2,pi/2,L, and pi/2-L where L is:
	#((2*pi*(1-wobbliness))/2)
	#Using trigonometry, we can calculate the radius to intersection in terms of the angle and circle radius:
	#intersect_point = radius * tan(L)
	#If intersect_point is greater than max_intersection, we're good!
	#Otherwise we want to cap it there.  So we find the inverse-function for "wobbliness"
	#1-atan2(intersect_point,radius)/math.pi = wobbliness
	max_intersection = 50
	angle_l = (2*math.pi*(1-wobbliness))/2
	intersect_point = radius * math.tan(angle_l)
	#print("Point of intersection: "  + str(intersect_point))
	if intersect_point > max_intersection:
		#print("Capping wobbliness prevent intersection!")
		wobbliness = 1-math.atan2(max_intersection,radius)/math.pi
		#print("New wobbliness: " + str(wobbliness))

	points = []
	lastnormal = None
	for a in range(0,switchbacknum):
		#Switchback starts with a circle, then a line
		#then we repeat
		
		circlesize = circlesize if uniformcircles else uniform(10,25)

		#cturn
		(generated, curpoint,deltalength,lastnormal) = generateConstantTurn(curpoint,circlesize,curtangent,
										circlepercent=wobbliness,turnleft=turnleft,
										turnagainstnormal=lastnormal)
		length += deltalength
		points.extend(generated)
		curtangent = endtangent(points)

		#straight
		(generated, curpoint,deltalength) = generateStraight(curpoint,straightsize,curtangent)
		length += deltalength
		points.extend(generated)
		curtangent = endtangent(points)


	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length)

def generateConstantTurnUntilFacingPoint(startpoint,radius,intangent,goalpoint,turnleft=None,turnagainstnormal=None,recursed=False):
	#This is a split-off version of generateConstantTurn, where instead of taking in a percent to turn around a circle,
	#We give it a direction we want it to stop at
	(startx,starty) = startpoint

	#cturns have a choices - turn left or turn right
	#Then, they can choose what percent of the circle do they want to turn?
	turnleft = uniform(0,1)<0.5 		if turnleft == None 		else turnleft

	#Calculating this is fairly complicated
	#Angle of circle normal = 90 degrees + intangent
	#Slope of normal = -tan(90+normal) if turn left or tan(90+normal) if turn right
	#Center is at startpoint + (r/sqrt( 1+m^2 )   ,   m*r*sqrt( 1+m^2 )) 
	circnorm = math.pi/2 + intangent
	slope    = math.tan(circnorm)
	purex = radius/math.sqrt( 1+slope*slope )
	if turnleft:
		purex*=-1

	centerx = startx + purex
	centery = starty + slope*purex

	#Now we use a rotation matrix to parameterize intermediate points:
	#Given start S and center C, any point on the circle angle A away is:
	#R_A[S-C] + C
	#Let us box this up in a nice function:
	def intermediatePoint(s,c,a):
		(sx,sy) = s
		(cx,cy) = c
		cosa    = math.cos(a)
		sina    = math.sin(a)
		delx    = sx-cx
		dely    = sy-cy
		resultx = cosa*delx-sina*dely+cx
		resulty = sina*delx+cosa*dely+cy
		return (resultx,resulty)

	def sgn(x):
		return -1 if x < 0 else 1 if x > 0 else 0

	angle = 2*math.pi
	flipper = -1 if intangent*purex > 0 else 1 #multiply by purex because we want to re-flip here if we flipped due to "turnleft"

	if turnagainstnormal != None:
		#In this case, we want to make sure it turns away from the inputted normal vector
		#So if normal=(a,b) we want to make sure its (-a,-b)
		#This is the same as changing turnleft
		#So first we want to compute the current normal to make sure we don't change anything
		#Current normal will be:
		#points[0][0]-centerx,points[0][1]-centery
		#So we pre-compute the points[0][a] since we haven't yet:
		points_0 = intermediatePoint(startpoint,(centerx,centery),0)
		curnormal = normalizevec((points_0[0]-centerx,points_0[1]-centery))
		turnagainstnormal = normalizevec(turnagainstnormal)
		
		#Due to floating point stuffs, we won't check direct equality, we'll just look at the sign!
		#We want normals to be flipped, so its bad if curnormal has the same sign!
		#We also only need to check both components just in the case where the normal is (0,y)
		needtoflip = abs(curnormal[0]-turnagainstnormal[0])<0.1 or abs(curnormal[1]-turnagainstnormal[1])<0.1

		if needtoflip:
			#print("Flipping!")
			centerx-=2*purex
			centery-=2*slope*purex
			flipper*=-1
	points = []
	rangemax = 5000
	stepsize = 1.0/rangemax
	for t in range(0,rangemax):
		points.append(intermediatePoint(startpoint,(centerx,centery),flipper*t*angle*stepsize))
		if t!=0:
			#Check if we're pointing in the right direction
			#This equates to checking if endpoint lies on the line at points[-1] with the appropriate tangent.
			(sx,sy) = points[-1]
			(ex,ey) = goalpoint
			appropriate_angle = endtangent(points)
			if t > 0.5*rangemax and not recursed:
				#Very big turn, we don't like that!  We'll just turn the other way instead
				newturnagainstnormal = None if turnagainstnormal == None else normalizevec((-turnagainstnormal[1],turnagainstnormal[0]))
				return generateConstantTurnUntilFacingPoint(startpoint,radius,intangent,goalpoint,turnleft=turnleft,
											turnagainstnormal=newturnagainstnormal,recursed=True)
			if abs(capangle(appropriate_angle) - capangle(math.atan2((ey-sy),(ex-sx)))) < 0.01:
				#Would do equality checking but limited precision, we just check if close!
				#If so, break as we've succeeded our task
				#print("Found goal at t=" + str(t))
				break

	#Length of circle is, fortunately, easy!  It's simply radius*angle
	length = angle*radius

	#Now we want to find the normal vector, because it's useful to have to determine whether it curves inwards or outwards
	#Normal vectors are always parallel to the vector from center to end point
	normal = (points[-1][0]-centerx,points[-1][1]-centery)

	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length,normal)


def generateConstantTurn(startpoint,radius,intangent,turnleft=None,circlepercent=None,turnagainstnormal=None):
	(startx,starty) = startpoint

	#cturns have a choices - turn left or turn right
	#Then, they can choose what percent of the circle do they want to turn?
	turnleft = uniform(0,1)<0.5 		if turnleft == None 		else turnleft
	circlepercent = uniform(0.1,0.5)	if circlepercent == None 	else circlepercent

	#Calculating this is fairly complicated
	#Angle of circle normal = 90 degrees + intangent
	#Slope of normal = -tan(90+normal) if turn left or tan(90+normal) if turn right
	#Center is at startpoint + (r/sqrt( 1+m^2 )   ,   m*r*sqrt( 1+m^2 )) 
	circnorm = math.pi/2 + intangent
	slope    = math.tan(circnorm)
	purex = radius/math.sqrt( 1+slope*slope )
	if turnleft:
		purex*=-1

	centerx = startx + purex
	centery = starty + slope*purex

	#Now we use a rotation matrix to parameterize intermediate points:
	#Given start S and center C, any point on the circle angle A away is:
	#R_A[S-C] + C
	#Let us box this up in a nice function:
	def intermediatePoint(s,c,a):
		(sx,sy) = s
		(cx,cy) = c
		cosa    = math.cos(a)
		sina    = math.sin(a)
		delx    = sx-cx
		dely    = sy-cy
		resultx = cosa*delx-sina*dely+cx
		resulty = sina*delx+cosa*dely+cy
		return (resultx,resulty)

	def sgn(x):
		return -1 if x < 0 else 1 if x > 0 else 0

	angle = 2*math.pi*circlepercent
	flipper = -1 if intangent*purex > 0 else 1 #multiply by purex because we want to re-flip here if we flipped due to "turnleft"

	if turnagainstnormal != None:
		#In this case, we want to make sure it turns away from the inputted normal vector
		#So if normal=(a,b) we want to make sure its (-a,-b)
		#This is the same as changing turnleft
		#So first we want to compute the current normal to make sure we don't change anything
		#Current normal will be:
		#points[0][0]-centerx,points[0][1]-centery
		#So we pre-compute the points[0][a] since we haven't yet:
		points_0 = intermediatePoint(startpoint,(centerx,centery),0)
		curnormal = normalizevec((points_0[0]-centerx,points_0[1]-centery))
		turnagainstnormal = normalizevec(turnagainstnormal)
		
		#Due to floating point stuffs, we won't check direct equality, we'll just look at the sign!
		#We want normals to be flipped, so its bad if curnormal has the same sign!
		#We also only need to check both components just in the case where the normal is (0,y)
		needtoflip = abs(curnormal[0]-turnagainstnormal[0])<0.1 or abs(curnormal[1]-turnagainstnormal[1])<0.1

		if needtoflip:
			#print("Flipping!")
			centerx-=2*purex
			centery-=2*slope*purex
			flipper*=-1

	points = [intermediatePoint(startpoint,(centerx,centery),flipper*t*angle*0.01) for t in range(0,100)]

	#Length of circle is, fortunately, easy!  It's simply radius*angle
	length = angle*radius

	#Now we want to find the normal vector, because it's useful to have to determine whether it curves inwards or outwards
	#Normal vectors are always parallel to the vector from center to end point
	normal = (points[-1][0]-centerx,points[-1][1]-centery)

	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length,normal)


def generateStraight(startpoint,length,angle):
	(startx,starty) = startpoint
	
	#Now create a parameterized function in terms of the angle
	#This is easy - tan(angle) = slope, so y = mt + starty, x = t + startx
	#The length of this line is delx^2+dely^2 = length^2, so (mt)^2+t^2 = length^2
	#implying t^2 = length^2/(1+m^2)
	#So t ranges from 0 to length/sqrt(1+m^2)
	slope = math.tan(angle)
	tmax = length/math.sqrt(1+slope*slope)

	if angle*slope < 0:
		#I don't actually know the geometrical reason why this is needed :/
		#But if you don't do this, sometimes the line points the wrong way!
		tmax *=-1
	
	#Since we draw the track by placing lines, we only need the endpoints of this!
	#(For other curves we approximate by a bunch of small lines, so we'd need full data)
	#However we actually don't want that because it will mess with the self-intersection-detection
	#later on.
	if tmax >= 0:
		points = [(t+startx,slope*t+starty) for t in range(0,int(math.ceil(tmax)))]
	else:
		points = [(-t+startx,-slope*t+starty) for t in range(0,int(math.ceil(-tmax)))]
	#points = [startpoint,(tmax+startx,slope*tmax+starty)]


	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length)

def convertPointsToAllPositive(xys):
	#If we went negative anywhere, shift everything over
	#Returns shifted points tupled with the range over which the points span
	#We also want everything converted to an integer!
	maxnegx = 0
	maxnegy = 0
	maxx    = 0
        maxy    = 0

	for point in xys:
		(x,y) = point
		maxnegx = min(x,maxnegx)
		maxnegy = min(y,maxnegy)
		maxx    = max(x,maxx)
		maxy    = max(y,maxy)

	newxys = []
	padding = 10
	for point in xys:
		(x,y) = point
		newxys.append((int(x-maxnegx)+padding,int(y-maxnegy)+padding))

	return (newxys,int(maxx-maxnegx)+2*padding,int(maxy-maxnegy)+2*padding)

def compactify_points(points):
	#Given a list of int points, if any two adjacent points are the same then remove one of them
	removelist = []
	prevpoint = (-10000,-10000)
	for a in range(0,len(points)):
		if (points[a] == prevpoint):
			removelist.append(a)
		prevpoint = points[a]
	for index in sorted(removelist,reverse=True):
		del points[index]
	return points

def check_if_overlap(points):
	#Naive check to see if track overlaps itself
	#(Won't catch overlaps due to track width, only if track center overlaps)
	points = points[:-5] #remove end points as in theory that should also be the start point
	#(I remove extra to be a little generous to it as a courtesy - I don't really care how well the
	#start loops to the end yet)
	
	#Now we want to fill in the diagonally-connected points, otherwise you can imagine
	#that two tracks moving diagonally opposite could cross eachother inbetween the pixels,
	#fooling our test.
	for index in range(1,len(points)):
		(sx,sy) = points[index-1]
		(ex,ey) = points[index]
		manhatdist = abs(ex-sx)+abs(ey-sy)
		if (manhatdist > 1):
			#moved diagonally, insert an extra point for it at the end!
			points.append( (sx+1,sy) if ex > sx else (sx-1,sy) )

	return len(set(points)) != len(points)

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
