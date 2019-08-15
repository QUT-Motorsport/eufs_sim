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
import pandas as pd

#Here are all the track formats we care about:
#.launch (well, we actually want the model data, not the .launch, but we'll treat it as wanting the .launch)
#		(since the end user shouldn't have to care about the distinction)
#.png
#.csv
#raw data ("xys",this will be hidden from the user as it is only used to convert into .pngs)
class ConversionTools:
	def __init__():
		pass

	# Define colors for track gen and image reading
	noisecolor = (0,255,255,255)	#cyan, the turquoise turqouise wishes it were
	bgcolor = (255,255,255,255)	#white
	conecolor  = (255,0,255,255)	#magenta, for yellow cones (oops...)
	conecolor2 = (0,0,255,255)	#blue, for blue cones
	carcolor = (0,255,0,255)	#green
	trackcenter = (0,0,0,255)	#black
	trackinner = (255,0,0,255)	#red
	trackouter = (255,255,0,255)	#yellow
	conecolorOrange = (255,165,0,255) #orange, for orange cones
	conecolorBigOrange = (127,80,0,255) #dark orange, for big orange cones	

	#Other various retained parameters
	linknum = -1

	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#This section handles Track Image metadata

	@staticmethod
	def getMetadataPixel(x,y,corner,pixels,size):
		#Returns the metadata pixel (x,y,corner), as specified by the Track Image specification on the team wiki.
		x,y = ConversionTools.getMetadataPixelLocation(x,y,corner,size)
		return pixels[x,y]

	@staticmethod
	def getMetadataPixelLocation(x,y,corner,size):
		#Returns the metadata pixel (x,y,corner), as specified by the Track Image specification on the team wiki.
		width = size[0]
		height = size[1]
		if corner == "Top Left":
			return (x,y)
		elif corner == "Bottom Left":
			return (x,height-6+y)
		elif corner == "Top Right":
			return (width-6+x,y)
		elif corner == "Bottom Right":
			return (width-6+x,height-6+x)


	@staticmethod
	def getRawMetadata(pixelvalue,mode="continuous"):
		#This function converts metadata as outlined in the specification for Track Images on the team wiki
		#It assumes that handling of the cases (255,255,255,255) and (r,g,b,0) are done outside this function.
		(r,g,b,a) = pixelvalue
		if mode == "continuous":
			return a-1 + (b-1)*254 + (g-1)*254**2 + (r-1)*254**3
		return None

	@staticmethod
	def convertScaleMetadata(pixelvalues):
		#This function converts the data obtained from scale metadata pixels into actual scale information
		#Output range is from 0.0001 to 100.
		
		primaryPixel = pixelvalues[0]
		secondaryPixel = pixelvalues[1]#unused in the specification

		if primaryPixel == (255,255,255,255) and secondaryPixel == (255,255,255,255): return 1 #Check for the default case

		metadata = ConversionTools.getRawMetadata(primaryPixel,mode="continuous")
		#Want to linearly transform the metadata, a range from 0 to 254**4-1, to the range 0.0001 to 100
		toReturn = metadata/(254**4-1.0) * (100-0.0001) + 0.0001
		return toReturn


	@staticmethod
	def deconvertScaleMetadata(data):
		#This function converts a raw scale value into a list of metadata pixels needed to replicate it.
		#First in list is the primary metadata pixel, second in list is the secondary (which is unused in the specification)
		metadata = int((data-0.0001)/(100-0.0001) * (254**4-1))
		a = metadata % 254
		metadata = (metadata - a) // 254
		b = metadata % 254
		metadata = (metadata - b) // 254
		g = metadata % 254
		r = (metadata - g) // 254

		primaryPixel = (r+1,g+1,b+1,a+1)
		secondaryPixel = (255,255,255,255)
		return [primaryPixel,secondaryPixel]


	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################

	@staticmethod
	def convert(cfrom,cto,what,params=[],conversion_suffix=""):
		if cfrom=="xys" and cto=="png":
			return ConversionTools.xys_to_png(what,params,conversion_suffix)
		if cfrom=="png" and cto=="launch":
			return ConversionTools.png_to_launch(what,params,conversion_suffix)
		if cfrom=="png" and cto=="csv":
			ConversionTools.png_to_launch(what,params,conversion_suffix)
			newwhatarray = what.split("/")
			newwhatarray[-2] = "launch"
			what = "/".join(newwhatarray)
			return ConversionTools.launch_to_csv(what[:-4]+conversion_suffix+".launch",params,conversion_suffix="")
		if cfrom=="launch" and cto=="csv":
			return ConversionTools.launch_to_csv(what,params,conversion_suffix)
		if cfrom=="launch" and cto=="png":
			ConversionTools.launch_to_csv(what,params,conversion_suffix)
			newwhatarray = what.split("/")
			newwhatarray[-2] = "tracks"
			what = "/".join(newwhatarray)
			return ConversionTools.csv_to_png(what[:-7]+conversion_suffix+".csv",params,conversion_suffix="")
		if cfrom=="csv" and cto == "launch":
			ConversionTools.csv_to_png(what,params,conversion_suffix)
			newwhatarray = what.split("/")
			newwhatarray[-2] = "randgen_imgs"
			what = "/".join(newwhatarray)
			return ConversionTools.png_to_launch(what[:-4]+conversion_suffix+".png",params,conversion_suffix="")
		if cfrom=="csv" and cto == "png":
			return ConversionTools.csv_to_png(what,params,conversion_suffix)
		if cto == "ALL":
			#Don't worry, if something tries to convert to itself it just gets ignored
			ConversionTools.convert(cfrom,"launch",what,params,conversion_suffix)
			ConversionTools.convert(cfrom,"csv",what,params,conversion_suffix)
			return ConversionTools.convert(cfrom,"png",what,params,conversion_suffix)
		return None
			
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################

	@staticmethod
	def xys_to_png(what,params,conversion_suffix=""):
		GENERATED_FILENAME = "rand" + conversion_suffix
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

		#Now we want to make all pixels bordering the track become magenta (255,0,255) - this will be our 'cone' color
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
	def png_to_launch(what,params=[0],conversion_suffix=""):
		#This is a fairly intensive process - we need:
		#	to put %FILENAME%.launch in eufs_gazebo/launch
		#	to put %FILENAME%.world in eufs_gazebo/world
		#	to put %FILENAME%/model.config and %FILENAME%/model.sdf in eufs_description/models

		#Our template files are stored in eufs_launcher/resource as:
		#	randgen_launch_template
		#       randgen_world_template
		#	randgen_model_template/model.config
		#	randgen_model_template/model.sdf
		GENERATED_FILENAME = what.split('/')[-1][:-4]+conversion_suffix#[:-4] to split off .png, looks like an emoji...
		im = Image.open(what)
		noiseLevel = params[0]
		pixels = im.load()

		#Let's get the scale metadata from the png:
		scaledata = ConversionTools.convertScaleMetadata([
							ConversionTools.getMetadataPixel(0,0,"Top Left",pixels,im.size),
							ConversionTools.getMetadataPixel(1,0,"Top Left",pixels,im.size)])
		#scaledata represents how big a pixel is.

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
		for i in range(im.size[0]):
			for j in range(im.size[1]):
				p = pixels[i,j]
				if isCarColor(p):
					launch_merged = str(xCoordTransform(i*scaledata)).join(launch_merged.split("%PLACEX%"))
					launch_merged = str(yCoordTransform(j*scaledata)).join(launch_merged.split("%PLACEY%"))
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
		sdf_orangeconemodel = "model://eufs_description/meshes/cone.dae".join(sdf_model_with_collisions.split("%MODELNAME%"))
		sdf_bigorangeconemodel = "model://eufs_description/meshes/cone_big.dae".join(sdf_model_with_collisions.split("%MODELNAME%"))

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
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_yellowconemodel,i*scaledata,j*scaledata)
				elif p == ConversionTools.conecolor2:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_blueconemodel,i*scaledata,j*scaledata)
				elif p == ConversionTools.conecolorOrange:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_orangeconemodel,i*scaledata,j*scaledata)
				elif p == ConversionTools.conecolorBigOrange:
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_bigorangeconemodel,i*scaledata,j*scaledata)
				elif p == ConversionTools.noisecolor and uniform(0,1)<noiseLevel:
					sdf_noisemodel = getRandomNoiseModel().join(sdf_model_with_collisions.split("%MODELNAME%"))
					sdf_allmodels = sdf_allmodels + "\n" + putModelAtPosition(sdf_noisemodel,i*scaledata,j*scaledata)

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
	def launch_to_csv(what,params=[0],conversion_suffix=""):
		filename = what.split("/")[-1].split(".")[0]
		cardata_reader = open(what)
		cardata = cardata_reader.read()
		cardata_reader.close()
		carx   = cardata.split("<arg name=\"x\" default=\"")[1].split("\"")[0]
		cary   = cardata.split("<arg name=\"y\" default=\"")[1].split("\"")[0]
		caryaw = cardata.split("<arg name=\"yaw\" default=\"")[1].split("\"")[0]
		midpoints=False
		if len(params) >= 2:
			midpoints = params[1]
		Track.runConverter(filename,midpoints=midpoints,car_start_data=("car_start",carx,cary,caryaw),conversion_suffix = conversion_suffix)

	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################
	#######################################################################################################################################################

	@staticmethod
	def csv_to_png(what,params,conversion_suffix=""):
		filename = what.split("/")[-1].split(".")[0]+conversion_suffix
		#We are merely going to open up the csv, read through all the lines, and round down the point to an integer.
		#(While preserving cone color).
		df = pd.read_csv(what)
		bluecones = df[df['tag']=="blue"]
		yellowcones = df[df['tag']=="yellow"]
		orangecones = df[df['tag']=="orange"]
		bigorangecones = df[df['tag']=="big_orange"]
		carloc = df[df['tag']=="car_start"]

		rawblue = []
		rawyellow = []
		raworange = []
		rawbigorange = []
		rawcarloc = (0,0,0,0)
		for bluecone in bluecones.itertuples():
			x = (bluecone[2])
			y = (bluecone[3])
			rawblue.append(("blue",x,y,0))

		for yellowcone in yellowcones.itertuples():
			x = (yellowcone[2])
			y = (yellowcone[3])
			rawyellow.append(("yellow",x,y,0))

		for orangecone in orangecones.itertuples():
			x = (orangecone[2])
			y = (orangecone[3])
			raworange.append(("orange",x,y,0))

		for bigorangecone in bigorangecones.itertuples():
			x = (bigorangecone[2])
			y = (bigorangecone[3])
			rawbigorange.append(("big_orange",x,y,0))

		for c in carloc.itertuples():
			rawcarloc = ("car",(c[2]),(c[3]),(c[4]))

		allcones = rawblue + rawyellow + raworange + rawbigorange + [rawcarloc]

		#Here we convert it all to positive
		minx = 100000
		miny = 100000
		maxx = -100000
		maxy = -100000
		for cone in allcones:
			if cone[1]<minx:
				minx = cone[1]
			if cone[2]<miny:
				miny = cone[2]
			if cone[1]>maxx:
				maxx = cone[1]
			if cone[2]>maxy:
				maxy = cone[2]

		#Here we figure out the track scaling by calculating the average smallest distance between cones
		totalxdistance = 0
		totalydistance = 0
		for cone1 in allcones:
			closestx = 10000
			closesty = 10000
			for cone2 in allcones:
				if cone1 == cone2: continue
				dx = abs(cone1[1]-cone2[1])
				dy = abs(cone1[2]-cone2[2])
				if dx < closestx: closestx = dx
				if dy < closesty: closesty = dy
			totalxdistance+=closestx
			totalydistance+=closesty
		
		#Our scale will strive to preserve distances as small as a quarter of the average distance.
		scaleDesired = min(totalxdistance,totalydistance)/(len(allcones)-1) * 0.25
		if scaleDesired < 0.0001: scaleDesired = 0.0001#Clamp scale to allowed values
		if scaleDesired > 100:    scaleDesired = 100
		scaleMetadata = ConversionTools.deconvertScaleMetadata(scaleDesired)
		
		finalcones = []
		twidth = int((maxx-minx+20)/scaleDesired)
		theight = int((maxy-miny+20)/scaleDesired)
		carx = int((rawcarloc[1]-minx+10)/scaleDesired)
		cary = int((rawcarloc[2]-miny+10)/scaleDesired)
		for cone in allcones:
			finalcones.append( (cone[0],int((cone[1]-minx+10)/scaleDesired),int((cone[2]-miny+10)/scaleDesired))   )

		#draw the track
		im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0)) 
		draw = ImageDraw.Draw(im)

		#Convert data to image format
		draw.polygon([(0,0),(twidth,0),(twidth,theight),(0,theight),(0,0)],fill='white')#background

		pixels = im.load()
		#conecolor is inside, conecolor2 is outside
		def getConeColor(conename):
			if  conename ==  "yellow":     return ConversionTools.conecolor
			elif conename == "blue":       return ConversionTools.conecolor2
			elif conename == "orange":     return ConversionTools.conecolorOrange
			elif conename == "big_orange": return ConversionTools.conecolorBigOrange
			return ConversionTools.conecolor
		for cone in finalcones:
			pixels[cone[1],cone[2]] = getConeColor(cone[0])
		pixelValue = int(rawcarloc[3]/(2*math.pi)*254+1) # it is *254+1 because we never want it to be 0
		if pixelValue > 255: pixelValue = 255
		if pixelValue <   1: pixelValue =   1
		pixels[carx,cary] = (ConversionTools.carcolor[0],ConversionTools.carcolor[1],ConversionTools.carcolor[2],pixelValue)
		
		#Add metadata:
		loc = ConversionTools.getMetadataPixelLocation(0,0,"Top Left",im.size)
		pixels[loc[0],loc[1]] = scaleMetadata[0]
		loc = ConversionTools.getMetadataPixelLocation(1,0,"Top Left",im.size)
		pixels[loc[0],loc[1]] = scaleMetadata[1]

		#Save it:
		im.save(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'randgen_imgs/'+filename+'.png'))
		return im
		
