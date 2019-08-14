from PIL import Image
from PIL import ImageDraw
import math
from TrackGenerator import endtangent as getTangentAngle
from random import randrange, uniform
import os
import rospkg
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

	@staticmethod
	def convert(cfrom,cto,what):
		if cfrom=="xys" and cto=="png":
			return ConversionTools.xys_to_png(what)


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
