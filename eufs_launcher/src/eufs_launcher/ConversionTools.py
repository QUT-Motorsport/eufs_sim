from PIL import Image
from PIL import ImageDraw
import math
from TrackGenerator import endtangent as getTangentAngle
from random import randrange, uniform
import os
import rospkg
import rospy
import sys
sys.path.insert(1, os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'tracks'))
from track_gen import Track

#Here are all the track formats we care about:
#.launch (well, we actually want the model data, not the .launch, but we'll treat it as wanting the .launch)
#.png
#.csv
#raw data ("xys",this will be hidden from the user as it is only used to convert into .pngs)
class ConversionTools:
	def __init__():
		pass

	# Define colors for track gen and image reading
	noisecolor = (0,255,255,255)	#cyan, the turquoise turqouise wishes it were
	bgcolor = (255,255,255,255)	#white
	conecolor  = (255,0,255,255)	#magenta, for yellow cones
	conecolor2 = (0,0,255,255)	#blue, for blue cones
	carcolor = (0,255,0,255)	#green
	trackcenter = (0,0,0,255)	#black
	trackinner = (255,0,0,255)	#red
	trackouter = (255,255,0,255)	#yellow

	#Other various retained parameters
	linknum = -1

	@staticmethod
	def convert(cfrom,cto,what,params=[]):
		if cfrom=="xys" and cto=="png":
			return ConversionTools.xys_to_png(what)
		if cfrom=="png" and cto=="launch":
			return ConversionTools.png_to_launch(what,params)
		if cfrom=="launch" and cto=="csv":
			return ConversionTools.launch_to_csv(what,params)
		if cfrom=="png" and cto=="csv":
			ConversionTools.png_to_launch(what,params)
			return ConversionTools.launch_to_csv(what[:-3]+"launch",params)
			
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################

	@staticmethod
	def xys_to_png(what):
		GENERATED_FILENAME = "rand"
		#Unpack
		(xys,twidth,theight) = what
		#Create image to hold data
		im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0)) 
		draw = ImageDraw.Draw(im)

		#Convert data to image format
		draw.polygon([(0,0),(twidth,0),(twidth,theight),(0,theight),(0,0)],fill='white')#background
		draw.line(xys,fill=ConversionTools.trackouter,width=5)#track full width
		draw.line(xys,fill=ConversionTools.trackinner,width=3)#track innards
		draw.line(xys,fill=ConversionTools.trackcenter)#track center


		pixels = im.load()#get reference to pixel data


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
		colorforcar = (ConversionTools.carcolor[0],ConversionTools.carcolor[1],ConversionTools.carcolor[2],pixelValue)

		draw.line([xys[0],xys[0]],fill=colorforcar)#car position

		def istrack(c):
			return c == ConversionTools.trackouter or c == ConversionTools.trackinner or c == ConversionTools.trackcenter

		#Now we want to make all pixels boardering the track become magenta (255,0,255) - this will be our 'cone' color
		#To find pixel boardering track, simply find white pixel adjacent to a non-white non-magenta pixle
		#We will also want to make it such that cones are about 4-6 away from eachother euclideanly	

		pixels = im.load()#get reference to pixel data

		prevPoint = (-10000,-10000)
		for i in range(len(xys)):
			if i == 0: continue #skip first part [as hard to calculate tangent]
			curPoint = xys[i]
			distanceVecFromPrevPoint = (curPoint[0]-prevPoint[0],curPoint[1]-prevPoint[1])
			distanceFromPrevPoint = distanceVecFromPrevPoint[0]**2 + distanceVecFromPrevPoint[1]**2
			if distanceFromPrevPoint < 4**2: continue#Skip if too close to previous point
			prevPoint = curPoint
			curTangentAngle = getTangentAngle(xys[:(i+1)])
			curTangentNormal = (5*math.sin(curTangentAngle),-5*math.cos(curTangentAngle))
			northPoint = ( int ( curPoint[0]+curTangentNormal[0] ) , int ( curPoint[1]+curTangentNormal[1] ) )
			southPoint = ( int ( curPoint[0]-curTangentNormal[0] ) , int ( curPoint[1]-curTangentNormal[1] ) )
			if not istrack(pixels[northPoint[0],northPoint[1]]):
				pixels[northPoint[0],northPoint[1]]=ConversionTools.conecolor
			if not istrack(pixels[southPoint[0],southPoint[1]]):
				pixels[southPoint[0],southPoint[1]]=ConversionTools.conecolor


		
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
				if pix == ConversionTools.conecolor:
					pixels[i,j] = ConversionTools.conecolor2
					newfrontier.update(getAllowedAdjacents(exploredlist,(i,j)))
				elif pix == ConversionTools.bgcolor:
					newfrontier.update(getAllowedAdjacents(exploredlist,(i,j)))
				exploredlist.add(f)
			frontier = newfrontier
			#curexplored = len(exploredlist)
			#maxexplored = twidth*theight*1.0
			#print("Max Percent: " + str(curexplored/maxexplored))
				
		

		#Finally, we just need to place noise.  At maximal noise, the track should be maybe 1% covered? (that's actually quite a lot!)

		for i in range(im.size[0]):
			for j in range(im.size[1]):
				if pixels[i,j] == ConversionTools.bgcolor:
					if uniform(0,100) < 1:#1% covered maximal noise
						pixels[i,j] = ConversionTools.noisecolor

		im.save(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+GENERATED_FILENAME+'.png'))
		return im

	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################

	@staticmethod
	def png_to_launch(what,params=[0]):
		#This is a fairly intensive process - we need:
		#	to put %FILENAME%.launch in eufs_gazebo/launch
		#	to put %FILENAME%.world in eufs_gazebo/world
		#	to put %FILENAME%/model.config and %FILENAME%/model.sdf in eufs_description/models

		#Our template files are stored in eufs_launcher/resource as:
		#	randgen_launch_template
		#       randgen_world_template
		#	randgen_model_template/model.config
		#	randgen_model_template/model.sdf
		GENERATED_FILENAME = what.split('/')[-1][:-4]#[:-4] to split off .png, looks like an emoji...
		im = Image.open(what)
		noiseLevel = params[0]
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
			return x[:-1]==ConversionTools.carcolor[:-1]
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
		ConversionTools.linknum = -1
		def putModelAtPosition(mod,x,y):
			ConversionTools.linknum+=1
			return str(ConversionTools.linknum).join( \
				str(xCoordTransform(x)).join( \
					str(yCoordTransform(y)).join( \
						mod.split("%PLACEY%")).split("%PLACEX%")).split("%LINKNUM%"))

		sdf_allmodels = ""
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				p = pixels[i,j]
				if p == ConversionTools.conecolor:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_yellowconemodel,i,j)
				elif p == ConversionTools.conecolor2:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_blueconemodel,i,j)
				elif p == ConversionTools.noisecolor and uniform(0,1)<noiseLevel:
					sdf_noisemodel = getRandomNoiseModel().join(sdf_model_with_collisions.split("%MODELNAME%"))
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_noisemodel,i,j)

		sdf_main = sdf_allmodels.join(sdf_main.split("%FILLDATA%"))

		sdf_out = open(os.path.join(rospkg.RosPack().get_path('eufs_description'), 'models',GENERATED_FILENAME,"model.sdf"),"w")
		sdf_out.write(sdf_main)
		sdf_out.close()
		



	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################

	@staticmethod
	def launch_to_csv(what,params=[0]):
		filename = what.split("/")[-1].split(".")[0]
		Track.runConverter(filename)
