import math
from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform
from rospy import loginfo as ROSLOG
from rospy import logerr as ROSERR


"""
To whomever in the future has come here to modify this code:
	I have tried to make this code as readable as possible, 
with plenty of comments explaining the mathematical reasoning 
behind the logic.
	However, the math behind it, while not conceptually complicated,
is a bit of a formula slog - and to make things worse, its very
geometric which is hard to convey through text.
	So, if you are a bit lost here, feel free to reach out to me (Bailey)
so that I can provide you with a better explanation of how things work.
	The entire track generator project can be thought of through a mathematical
lense as the following problem: Using only line and circle segments, how can we combine
them into a G1-continuous closed loop satisfying arbitrary restraints on the size of said
components?  
	The grand majority of the math is "tangent-bookeeping", that is to say that
we are primarily concerned with calculating and maintaining the consistency of
tangent lines.
"""



class TrackGenerator:

	#The rules for autocross:
	#	Straights: 		<=80 meters
	#	Constant Turns:		<=25 meter radius
	#	Hairpin Turns: 		>=4.5 meter outside radius
	#	Slalom: 		Cones in a straight line with 7.5 to 12 meter spacing [NOTE: can't be generated at this point, added later]
	#	Track Width: 		>=3 meters
	#	Track Length: 		<=1500 meters

	MIN_STRAIGHT = 20
	MAX_STRAIGHT = 80
	MIN_CONSTANT_TURN = 10
	MAX_CONSTANT_TURN = 25
	MIN_HAIRPIN = 4.5
	MAX_TRACK_LENGTH = 1500

	def __init__(self):
		pass

	@staticmethod
	def getpresets():
		return [("Contest Rules",[
			10,#Min straight length
			80,#Max straight length
			10,#Min constant turn radius
			25,#Max constant turn radius
			4.5,#Min hairpin turn radius
			10,#Max hairpin turn radius
			1500#Max length
			]),
			("Small Straights",[
			5,#Min straight length
			40,#Max straight length
			10,#Min constant turn radius
			25,#Max constant turn radius
			4.5,#Min hairpin turn radius
			10,#Max hairpin turn radius
			1500#Max length
			])]

	@staticmethod
	def getpreset(name):
		allPresets = TrackGenerator.getpresets()
		for a in allPresets:
			if a[0] == name:
				return a[1]
		ROSERR("No such preset: " + name)
		ROSERR("Defaulting to Contest Rules")
		return getpreset("Contest Rules")

	@staticmethod
	def setpreset(name):
		values = TrackGenerator.getpreset(name)
		TrackGenerator.MIN_STRAIGHT = values[0]
		TrackGenerator.MAX_STRAIGHT = values[1]
		TrackGenerator.MIN_CONSTANT_TURN = values[2]
		TrackGenerator.MAX_CONSTANT_TURN = values[3]
		TrackGenerator.MIN_HAIRPIN = values[4]
		TrackGenerator.MAX_HAIRPIN = values[5]
		TrackGenerator.MAX_TRACK_LENGTH = values[6]

	@staticmethod
	def generate(preset):
		#Generate the track as pure data
		#Returns a list of points to define the path of the track, along with a bounding width & height for how big the track is.
		TrackGenerator.setpreset(preset)
		xys = []
		overlapped = False
		while overlapped or xys==[]:
			#Re-generate if the track overlaps itself
			(xys,twidth,theight) = generateAutocrossTrackdriveTrack((0,0))
			xys = compactify_points(xys)
			overlapped = check_if_overlap(xys)
			if overlapped:
				print("Oops!  The track intersects itself too much.  Retrying...")
		return (xys,twidth,theight)

	

"""
ESSENTIAL FUNCTIONS
"""

def endtangent(xys):
	#Calculate direction of outgoing tangent of a set of points
	#Is an angle!
	sx = xys[-2][0]
	sy = xys[-2][1]
	ex = xys[-1][0]
	ey = xys[-1][1]
	return math.atan2((ey-sy),(ex-sx))


def generateAutocrossTrackdriveTrack(startpoint):
		
		xys = []
		curTrackLength = 0
		curpoint = startpoint

		#Let's start with a small straght
		startangle = math.pi/8
		(generated, curpoint, deltalength) = generateStraight(startpoint,TrackGenerator.MIN_STRAIGHT,startangle)
		curTrackLength += deltalength
		xys.extend(generated)

		#Now we want to set checkpoints to pass through:
		goalpoints = [	(startpoint[0]+TrackGenerator.MAX_TRACK_LENGTH*0.08,startpoint[1]),
				(startpoint[0]+TrackGenerator.MAX_TRACK_LENGTH*0.12,startpoint[1]+TrackGenerator.MAX_TRACK_LENGTH*0.08),
				(startpoint[0]-TrackGenerator.MAX_TRACK_LENGTH*0.03,startpoint[1]+TrackGenerator.MAX_TRACK_LENGTH*0.12)]
		for goalpoint in goalpoints:
			(generated, curpoint, length) = generatePathFromPointToPoint(curpoint,goalpoint,endtangent(xys),fuzzradius=20)
			curTrackLength+= length
			xys.extend(generated)

		#Now lets head back to the start:
		#We're gonna set a checkpoint that is close but not exactly the start point
		#so that we have breathing room for the final manouever:
		(generated, curpoint, length) = generatePathFromPointToPoint(curpoint,\
										(startpoint[0]-TrackGenerator.MAX_STRAIGHT*0.5,\
										startpoint[1]+TrackGenerator.MAX_CONSTANT_TURN*1.5),\
										endtangent(xys),fuzzradius=0)
		curTrackLength+= length
		xys.extend(generated)

		#Now we will add a circle to point directly away from the startpoint
		goalTangent = (-math.cos(startangle),-math.sin(startangle))
		goalPoint = startpoint
		initialTangentAngle = endtangent(xys)
		initialTangent = (math.cos(initialTangentAngle),math.sin(initialTangentAngle))
		initialPoint = (xys[-1][0],xys[-1][1])
		outerTurnAngle = math.acos(  -initialTangent[0]*goalTangent[0] - initialTangent[1]*goalTangent[1]  )
		circleTurnAngle = math.pi - outerTurnAngle
		circleTurnPercent = circleTurnAngle / (2*math.pi)
		circleRadius = TrackGenerator.MAX_CONSTANT_TURN
		(generated,curpoint,length,outnormal) = generateConstantTurn(initialPoint,circleRadius,initialTangentAngle,circlepercent=circleTurnPercent,turnleft=True)
		curTrackLength+=length
		xys.extend(generated)

		#Add a circle to turn 180 degrees to face the start point directly
		#Radius is calculated by finding distance when projected along the normal
		outnormal = normalizevec(outnormal)
		diff = ( curpoint[0]-startpoint[0],curpoint[1]-startpoint[1] )
		circleRadius = (diff[0]*outnormal[0]+diff[1]*outnormal[1])/2
		(generated, curpoint, length, _) = generateConstantTurn(curpoint,circleRadius,endtangent(xys),circlepercent=0.5,turnleft=True)
		curTrackLength+=length
		xys.extend(generated)

		#Finally, add a straight to connect it to the start
		straightLength = magnitude( ( xys[-1][0] - startpoint[0], xys[-1][1] - startpoint[1] ) )*1.1
		(generated, curpoint, deltalength) = generateStraight(curpoint, straightLength ,endtangent(xys))
		curTrackLength += deltalength
		xys.extend(generated)

		#Check if accidentally created too big of a straight
		if straightLength + TrackGenerator.MIN_STRAIGHT > TrackGenerator.MAX_STRAIGHT:
			#We always start each track with a minimum-length straight, which is joined up with the final straight,
			#hence the addition of MIN_STRAIGHT here.
			print("Track gen failed - couldn't connect ending and still follow the preset rules!  Retrying.")
			return generateAutocrossTrackdriveTrack(startpoint)
		elif curTrackLength > 1500:
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
	circleradius = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)



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

	#Check if we're within MAX_STRAIGHT of the goal:
	(cx,cy) = points[-1]
	(ex,ey) = endpoint
	squaredistance = (ex-cx)*(ex-cx)+(ey-cy)*(ey-cy)
	#print("--------------------------------")
	#print(math.sqrt(squaredistance))
	#print(points[-1])
	#print(endpoint)
	#print("++++++++++++++++++++++++++++++++")
	if squaredistance <= TrackGenerator.MAX_STRAIGHT**2+fuzzradius**2:
		#We'll just draw a straight to it
		(generated, curpoint,deltalength) = generateStraight(points[-1],min(math.sqrt(squaredistance),TrackGenerator.MAX_STRAIGHT),endtangent(points))
		length += deltalength
		points.extend(generated)
	else:
		#Go as far as we can (unless we're very close in which case don't, because it'll cause the next iteration to look weird)
		straightsize = TrackGenerator.MAX_STRAIGHT if squaredistance <= (TrackGenerator.MAX_STRAIGHT*1.2)**2 else TrackGenerator.MAX_STRAIGHT/2
		(generated, curpoint,deltalength) = generateStraight(points[-1],straightsize,endtangent(points))
		length += deltalength
		points.extend(generated)
		#We'll either do a random cturn or a random hairpin, then continue the journey
		cturn_or_hairpin = uniform(0,1)
		makecturn = cturn_or_hairpin < 0.7
		if makecturn or (hairpined and not manyhairpins):#cturn
			(generated, curpoint,deltalength,output_normal) = generateConstantTurn(curpoint,
											uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN),
											endtangent(points),
											circlepercent=uniform(0,0.25),turnagainstnormal = output_normal)
			length += deltalength
			points.extend(generated)
			(generated, curpoint,deltalength,output_normal) = generateConstantTurn(curpoint,
											uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN),
											endtangent(points),
											circlepercent=uniform(0,0.25),turnagainstnormal = output_normal)
			length += deltalength
			points.extend(generated)
		else:
			#We only want an even amount of turns so that we can leave heading the same direction we entered.
			#otherwise due to the way we head towards the path, its basically guaranteed we get a self-intersection.
			numswitches = 2*randrange(1,3)
			(generated, curpoint,deltalength) = generateHairpinTurn(curpoint,uniform(TrackGenerator.MIN_HAIRPIN,TrackGenerator.MAX_HAIRPIN),
										endtangent(points),switchbacknum=numswitches)
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
	turnleft = uniform(0,1)<0.5 									if turnleft == None 		else turnleft
	switchbacknum = randrange(2,10)									if switchbacknum == None	else switchbacknum
	wobbliness = uniform(0.45,0.55)									if wobbliness == None		else wobbliness
	straightsize = uniform(TrackGenerator.MIN_STRAIGHT,TrackGenerator.MAX_STRAIGHT)			if straightsize == None		else straightsize
	circlesize = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)		if circlesize == None		else circlesize

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
	
		circlesize = circlesize if uniformcircles else uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)

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

	points = [intermediatePoint(startpoint,(centerx,centery),flipper*t*angle*0.001) for t in range(0,1000)]

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
	#If the track dips to the negative side of the x or y axes, shift everything over
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
	points = points[:-10] #remove end points as in theory that should also be the start point
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



"""
HELPER FUNCTIONS

:Functions that save time but are not considered 'fundamental' to the process of track generation
"""

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

def magnitude(vec):
	(a,b) = vec
	return math.sqrt(a*a+b*b)

def normalizevec(vec):
	(a,b) = vec
	mag = math.sqrt(a*a+b*b)
	return (a/mag,b/mag)
