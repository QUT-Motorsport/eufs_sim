import math
from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform
import rospy
from scipy.special import binom
from LauncherUtilities import calculate_tangent_angle, cap_angle, cap_angle_odd, magnitude, normalize_vec
import numpy as np

"""
###############################################################################
###############################################################################
###############################################################################
###########################Track Generator Overview############################
###############################################################################
###############################################################################
###############################################################################
###########################Functions For Public Use############################
###############################################################################
#									      #
#	TrackGenerator.generate(values: List[float])                          #
#			-> Tuple[List[Tuple[float,float]],int,int]            #
#		Takes in a list of preset data.  Check out the get_presets()  #
#		function for a thorough and perpetually up to date list of    #
#		each piece of data.					      #
#		This returns a tuple of the form (xys,width,height), where:   #
#			xys:						      #
#				A list of points that outline the	      #
#				generated track.			      #
#			width:						      #
#				The width that is spanned by xys	      #
#			height:						      #
#				The height that is spanned by xys	      #
#		There will always be a margin of at least 5 on the returned   #
#		xys, so width and height are always at least 10 larger than   #
#		if the range was strictly calculated.			      #
#		This is done for the sake of ConversionTools' TrackImage      #
#		specification, which requires the margin.		      #
#									      #
#	TrackGenerator.get_presets()  -> List[Tuple[str,List[float]]]         #
#		Returns a list of all generator presets (the str in the tuple)#
#		coupled with their preset values (the list in the tuple)      #
#									      #
#	TrackGenerator.get_default_preset() -> str			      #
#		Returns the default preset (usually "Small Straights")        #
#									      #
#	TrackGenerator.get_preset_names() -> List[str]                        #
#		Returns a list of all the generator presets.                  #
#									      #
#	TrackGenerator.get_preset(name: str) -> List[float]		      #
#		Returns a list of all the preset data of the specified        #
#		preset (the one with the matching name).                      #
#									      #
###############################################################################
#                                                                             #
#	There are other functions available (many others), but they are not   #
#	necessarily meant for use outside this file.                          #
#									      #
#	That doesn't stop them from being used, of course - but depending on  #
#	the circumstances, their use implies that they maybe should be        #
#	factored out into a seperate library.				      #
#									      #
#	Since the LauncherModule/TrackGenerator/ConversionTools ecosystem is  #
#	still in a state of flux, function signatures for all functions but   #
#	the ones listed above are subject to change without warning.          #
#									      #
###############################################################################
"""

class GenerationFailedException(Exception):
   """Raised when generator takes too long."""
   pass

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
	LAX_GENERATION = False
	TRACK_MODE = "Circle&Line"
	FAILURE_INFO = False #debugging parameter to print out when generation fails

	def __init__(self):
		pass

	@staticmethod
	def get_presets():
		return [("Contest Rules",[
				10,#Min straight length
				80,#Max straight length
				10,#Min constant turn radius
				25,#Max constant turn radius
				4.5,#Min hairpin turn radius
				10,#Max hairpin turn radius
				3,#Max hairpin pairs amount
				1500,#Max length
				0,#Lax Generation (off)
				0#Circle&Line mode
			]),
			("Small Straights",[
				5,#Min straight length
				40,#Max straight length
				10,#Min constant turn radius
				25,#Max constant turn radius
				4.5,#Min hairpin turn radius
				10,#Max hairpin turn radius
				3,#Max hairpin pairs amount
				700,#Max length
				1,#Lax Generation (on)
				0#Circle&Line mode
			]),
			("Computer Friendly",[
				10,#Min straight length
				80,#Max straight length
				5,#Min constant turn radius
				15,#Max constant turn radius
				4.5,#Min hairpin turn radius
				10,#Max hairpin turn radius
				3,#Max hairpin pairs amount
				500,#Max length
				1,#Lax Generation (on)
				0#Circle&Line mode
			]),
			("Bezier",[
				10,#Min straight length
				80,#Max straight length
				5,#Min constant turn radius
				15,#Max constant turn radius
				4.5,#Min hairpin turn radius
				10,#Max hairpin turn radius
				3,#Max hairpin pairs amount
				500,#Max length
				1,#Lax Generation (on)
				1#Bezier mode
			])]

	@staticmethod
	def get_default_mode_string():
		return "Circle&Line"

	@staticmethod
	def get_default_mode_number():
		return TrackGenerator.get_number_from_mode(TrackGenerator.get_default_mode_string())

	@staticmethod
	def get_mode_from_number(num):
		if num==0: return "Circle&Line"
		elif num==1: return "Bezier"
		return "Circle&Line"

	@staticmethod
	def get_number_from_mode(mode):
		if mode=="Circle&Line": return 0
		elif mode == "Bezier": return 1
		return 0

	@staticmethod
	def get_default_preset():
		return "Small Straights"

	@staticmethod
	def get_preset_names():
		to_return = []
		all_presets = TrackGenerator.get_presets()
		for a in all_presets:
			to_return.append(a[0])
		return to_return

	@staticmethod
	def get_preset(name):
		all_presets = TrackGenerator.get_presets()
		for a in all_presets:
			if a[0] == name:
				return a[1]
		rospy.logerr("No such preset: " + name)
		rospy.logerr("Defaulting to Contest Rules")
		return get_preset("Contest Rules")

	@staticmethod
	def set_preset(name):
		values = TrackGenerator.get_preset(name)
		TrackGenerator.set_data(values)
		

	@staticmethod
	def set_data(values):
		TrackGenerator.MIN_STRAIGHT = values[0]
		TrackGenerator.MAX_STRAIGHT = values[1]
		TrackGenerator.MIN_CONSTANT_TURN = values[2]
		TrackGenerator.MAX_CONSTANT_TURN = values[3]
		TrackGenerator.MIN_HAIRPIN = values[4]
		TrackGenerator.MAX_HAIRPIN = values[5]
		TrackGenerator.MAX_HAIRPIN_NUM = values[6]
		TrackGenerator.MIN_HAIRPIN_NUM = 1 if TrackGenerator.MAX_HAIRPIN_NUM > 0 else 0
		TrackGenerator.MAX_TRACK_LENGTH = values[7]
		TrackGenerator.LAX_GENERATION = values[8]==1
		TrackGenerator.TRACK_MODE = TrackGenerator.get_mode_from_number(values[9])

	@staticmethod
	def generate(values):
		#Generate the track as pure data
		#Returns a list of points to define the path of the track, along with a bounding width & height for how big the track is.
		#Input is a list of track parameters
		TrackGenerator.set_data(values)
		xys = []
		overlapped = False
		generate_function = generate_autocross_trackdrive_track if TrackGenerator.TRACK_MODE == "Circle&Line" else generate_bezier_track
		failure_count = 0
		while overlapped or xys==[]:
			#break
			#Re-generate if the track overlaps itself
			(xys,twidth,theight) = generate_function((0,0))
			xys2 = [(int(x[0]),int(x[1])) for x in xys]
			xys2 = compactify_points(xys2)
			overlapped = check_if_overlap(xys2)
			if overlapped:
				#break
				if TrackGenerator.FAILURE_INFO: rospy.logerr("Overlap check "+str(failure_count)+" failed")
				print("Oops!  The track intersects itself too much.  Retrying...")
				failure_count+=1
			if failure_count > 10000:
				raise GenerationFailedException
		
		#Now let's randomly flip it a bit to spice it up
		if uniform(0,1) < 0.5:#flip xys by x
			xys = [(-x+twidth,y) for (x,y) in xys]
		if uniform(0,1) < 0.5:#flip xys by y
			xys = [(x,-y+theight) for (x,y) in xys]

		return (xys,twidth,theight)

	

"""
ESSENTIAL FUNCTIONS
"""

def generate_bezier_track(start_point):
		#In this function we handle the quick&dirty Bezier generator
		xys = []

		goal_points = [	(start_point[0]+TrackGenerator.MAX_TRACK_LENGTH*0.08,start_point[1]),
				(start_point[0]+TrackGenerator.MAX_TRACK_LENGTH*0.12,start_point[1]+TrackGenerator.MAX_TRACK_LENGTH*0.08),
				(start_point[0]-TrackGenerator.MAX_TRACK_LENGTH*0.03,start_point[1]+TrackGenerator.MAX_TRACK_LENGTH*0.12)]

		bezier_out,tangent_in,tangent_out = get_random_bezier(start_point,goal_points[0])
		xys.extend([bezier_out(t*0.01) for t in range(0,101)])
		initial_tangent = tangent_in

		prev_tangent = tangent_out
		for g in range(1,len(goal_points)):
			bezier_out,tangent_in,prev_tangent = get_random_bezier(goal_points[g-1],goal_points[g],start_tangent=prev_tangent)
			xys.extend([bezier_out(t*0.01) for t in range(0,101)])

		bezier_out,tangent_in,prev_tangent = get_random_bezier(goal_points[-1],start_point,start_tangent=prev_tangent,calculate_tangent_angle=initial_tangent)
		xys.extend([bezier_out(t*0.01) for t in range(0,101)])
		
		return convert_points_to_all_positive(xys)

def get_random_bezier(start_point,end_point,start_tangent=None,calculate_tangent_angle=None,order=4):
		#For the math to work out, we need Beziers to be at least quartic
		start_tangent = uniform(0,2*math.pi) if start_tangent == None else start_tangent
		calculate_tangent_angle   = uniform(0,2*math.pi) if calculate_tangent_angle   == None else calculate_tangent_angle

		#The incoming tangent is the same as the line from P0 to P1
		#Outgoing tangent is the same as the line from P(n-1) to P(n)
		#Where P0, P1, ..., P(n-1), P(n) are the control points
		#All other control points are free to be selected.
		scale = uniform(10,100)
		p0_to_p1 = (math.cos(start_tangent)*scale,math.sin(start_tangent)*scale)
		p0 = start_point
		p1 = (p0[0]+p0_to_p1[0],p0[1]+p0_to_p1[1])

		scale = uniform(10,100)
		pn_1_to_pn = (math.cos(calculate_tangent_angle)*scale,math.sin(calculate_tangent_angle)*scale)
		pn = end_point
		pn_1 = (pn[0]-pn_1_to_pn[0],pn[1]-pn_1_to_pn[1])

		control_points = [p0,p1,pn_1,pn]

		return (get_parametric_bezier(control_points),start_tangent,calculate_tangent_angle)


def get_parametric_bezier(control_points):
		#This function will itself return a function of a parameterized bezier
		#That is, the result will be a function that takes a time parameter from 0 to 1
		#and traveling along it results in the points on the bezier.
		#I made this code match the Bezier curve definition on wikipedia as closely as
		#possible (Explicit definition, not the recursive one)
		def to_return(cp,t):
			the_sum_x = 0
			the_sum_y = 0
			n = len(cp)
			for i in range(n):
				coefficient = binom(n-1,i) * (1-t)**(n-i-1) * t**i
				the_sum_x += coefficient * cp[i][0]
				the_sum_y += coefficient * cp[i][1]
			return (the_sum_x,the_sum_y)
		return lambda t: to_return(control_points,t)

def generate_autocross_trackdrive_track(start_point):
		#In this function we handle the traditional Circle&Line generator
		xys = []
		cur_track_length = 0
		cur_point = start_point

		#Let's start with a small straght
		start_angle = uniform(0,math.pi/8)
		(generated, cur_point, delta_length) = generate_straight(start_point,TrackGenerator.MIN_STRAIGHT,start_angle)
		cur_track_length += delta_length
		xys.extend(generated)

		#Now we want to set checkpoints to pass through:
		goal_points = [	(start_point[0]+TrackGenerator.MAX_TRACK_LENGTH*0.08,start_point[1]),
				(start_point[0]+TrackGenerator.MAX_TRACK_LENGTH*0.12,start_point[1]+TrackGenerator.MAX_TRACK_LENGTH*0.08),
				(start_point[0]-TrackGenerator.MAX_TRACK_LENGTH*0.03,start_point[1]+TrackGenerator.MAX_TRACK_LENGTH*0.12)]

		fails = 0
		max_fails = 1#This controls how much it tries to salvage a bad run
				#It turns out that most times it fails, its not salvageable,
				#so I set it to 1 so that as soon as it fails it scraps the run.
		for goal_point in goal_points:
			prev_xys = xys
			(generated, cur_point, length) = generate_path_from_point_to_point(cur_point,goal_point,calculate_tangent_angle(xys),fuzz_radius=20)
			cur_track_length += length
			xys.extend(generated)
			#Now let's do early-checking for overlaps
			test = compactify_points([(int(x[0]),int(x[1])) for x in xys])
			if check_if_overlap(test): 
				if TrackGenerator.FAILURE_INFO: rospy.logerr("Early Overlap Checking: Failed")
				fails+=1
				cur_track_length -= length
				xys = prev_xys
				if fails==max_fails:
					return (test,0,0)

		#Now lets head back to the start:
		#We're gonna set a checkpoint that is close but not exactly the start point
		#so that we have breathing room for the final manouever:
		(generated, cur_point, length) = generate_path_from_point_to_point(cur_point,\
										(start_point[0]-TrackGenerator.MAX_STRAIGHT*0.5,\
										start_point[1]+TrackGenerator.MAX_CONSTANT_TURN*2),\
										calculate_tangent_angle(xys),fuzz_radius=0)
		cur_track_length+= length
		xys.extend(generated)

		#Now we will add a circle to point directly away from the start_point
		goal_tangent = (-math.cos(start_angle),-math.sin(start_angle))
		goal_point = start_point
		initial_tangent_angle = calculate_tangent_angle(xys)
		initial_tangent = (math.cos(initial_tangent_angle),math.sin(initial_tangent_angle))
		initial_point = (xys[-1][0],xys[-1][1])
		outer_turn_angle = math.acos(  -initial_tangent[0]*goal_tangent[0] - initial_tangent[1]*goal_tangent[1]  )
		circle_turn_angle = math.pi - outer_turn_angle
		circle_turn_percent = circle_turn_angle / (2*math.pi)
		circle_radius = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)
		(generated,cur_point,length,normal_out) = generate_constant_turn(
								initial_point,
								circle_radius,
								initial_tangent_angle,
								circle_percent=circle_turn_percent,
								turn_left=True)
		cur_track_length+=length
		xys.extend(generated)

		#Add a circle to turn 180 degrees to face the start point directly
		#Radius is calculated by finding distance when projected along the normal
		normal_out = normalize_vec(normal_out)
		diff = ( cur_point[0]-start_point[0],cur_point[1]-start_point[1] )
		circle_radius2 = (diff[0]*normal_out[0]+diff[1]*normal_out[1])/2
		(generated, cur_point, length, _) = generate_constant_turn(
			cur_point,
			abs(circle_radius2),
			calculate_tangent_angle(xys),
			circle_percent=0.5,
			turn_left=circle_radius2>0)
		cur_track_length+=length
		xys.extend(generated)

		#Finally, add a straight to connect it to the start
		straight_length = magnitude( ( xys[-1][0] - start_point[0], xys[-1][1] - start_point[1] ) )*1.1
		(generated, cur_point, delta_length) = generate_straight(cur_point, straight_length ,calculate_tangent_angle(xys))
		cur_track_length += delta_length
		xys.extend(generated)

		if not TrackGenerator.LAX_GENERATION:
			#Check if accidentally created too big of a straight
			if straight_length + TrackGenerator.MIN_STRAIGHT > TrackGenerator.MAX_STRAIGHT:
				#We always start each track with a minimum-length straight, which is joined up with the final straight,
				#hence the addition of MIN_STRAIGHT here.
				print("Track gen failed - couldn't connect ending and still follow the preset rules!  Retrying.")
				return generate_autocross_trackdrive_track(start_point)
			elif cur_track_length > 1500:
				print("Track gen failed - track too long, oops!  Retrying.")
				return generate_autocross_trackdrive_track(start_point)

		return convert_points_to_all_positive(xys)


def generate_path_from_point_to_point(start_point,end_point,tangent_in,depth=20,hairpined=False,many_hairpins=False,fuzz_radius=0):
	#Here we want to get from a to b by randomly placing paths 
	#[Note: depth parameter is just to limit recursion overflows]
	#[And hairpined parameter prevents multiple hairpins - we should have at most
	#one or else its hard to generate nice paths]
	#[many_hairpins overrides this and allows an arbitrary amount]
	#[fuzz_radius is how close to the end we want to be]
	length = 0
	points = []
	circle_radius = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)

	#We want to know ahead of time which way the circle will turn - that way we don't get loopty-loops
	#where the circle turns nearly all the way around when it would have been better to have the center
	#on the other side.
	#We'll do this by calculating the normal we want.
	#First, we want to know the path from start to goal.
	#And more specifically, its angle
	direct_path_angle = calculate_tangent_angle([start_point,end_point])

	#Our circle code takes in "turn_against_normal" - alternatively, this parameter is equivalent to passing in
	#a vector pointing TOWARDS the radius.  So let's calculate that!  If we have the goal be at a "higher" angle
	#than it, we want the radius to be up, otherwise down.  The angle of it pointing up is simply pi/2 higher than the tangent (90 degrees)
	normal_angle = cap_angle(math.pi/2 + tangent_in)

	#We now calculate if we need to add an additional pi to it.
	#If direct_path_angle is higher than tangent_in - but how do we define higher?  We say that it is higher if the
	#counterclockwise angle difference is smaller than the clockwise difference.
	if cap_angle(direct_path_angle-normal_angle)>cap_angle(direct_path_angle-direct_path_angle):
		normal_angle = cap_angle(math.pi + normal_angle)

	#Also flip it if tangent heading in 'negative' direction
	#if (abs(cap_angle_odd(tangent_in))math.pi/2):
	#	normal_angle = cap_angle(math.pi + normal_angle)

	#Finally lets convert this into a normal:
	the_normal = (1,math.tan(normal_angle))

	#Now let's actually draw the circle!
	(generated, cur_point,delta_length,output_normal) = generate_constant_turn_until_facing_point(start_point,circle_radius,tangent_in,end_point,
													turn_against_normal = the_normal)
	length += delta_length
	points.extend(generated)

	#Check if we're within MAX_STRAIGHT of the goal:
	(cx,cy) = points[-1]
	(ex,ey) = end_point
	square_distance = (ex-cx)*(ex-cx)+(ey-cy)*(ey-cy)
	#print("--------------------------------")
	#print(math.sqrt(square_distance))
	#print(points[-1])
	#print(end_point)
	#print("++++++++++++++++++++++++++++++++")
	if square_distance <= TrackGenerator.MAX_STRAIGHT**2+fuzz_radius**2:
		#We'll just draw a straight to it
		(generated, cur_point,delta_length) = generate_straight(points[-1],
								min(math.sqrt(square_distance),TrackGenerator.MAX_STRAIGHT),
								calculate_tangent_angle(points))
		length += delta_length
		points.extend(generated)
	else:
		#Go as far as we can (unless we're very close in which case don't, because it'll cause the next iteration to look weird)
		straight_size = TrackGenerator.MAX_STRAIGHT if square_distance <= (TrackGenerator.MAX_STRAIGHT*1.2)**2 else TrackGenerator.MAX_STRAIGHT/2
		(generated, cur_point,delta_length) = generate_straight(points[-1],straight_size,calculate_tangent_angle(points))
		length += delta_length
		points.extend(generated)
		#We'll either do a random cturn or a random hairpin, then continue the journey
		cturn_or_hairpin = uniform(0,1)
		make_cturn = cturn_or_hairpin < 0.7
		if make_cturn or (hairpined and not many_hairpins):#cturn
			(generated, cur_point,delta_length,output_normal) = generate_constant_turn(cur_point,
											uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN),
											calculate_tangent_angle(points),
											circle_percent=uniform(0,0.25),turn_against_normal = output_normal)
			length += delta_length
			points.extend(generated)
			(generated, cur_point,delta_length,output_normal) = generate_constant_turn(cur_point,
											uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN),
											calculate_tangent_angle(points),
											circle_percent=uniform(0,0.25),turn_against_normal = output_normal)
			length += delta_length
			points.extend(generated)
		else:
			#We only want an even amount of turns so that we can leave heading the same direction we entered.
			#otherwise due to the way we head towards the path, its basically guaranteed we get a self-intersection.
			num_switches = 2*randrange(TrackGenerator.MIN_HAIRPIN_NUM,TrackGenerator.MAX_HAIRPIN_NUM)
			(generated, cur_point,delta_length) = generate_hairpin_turn(cur_point,uniform(TrackGenerator.MIN_HAIRPIN,TrackGenerator.MAX_HAIRPIN),
										calculate_tangent_angle(points),switchback_num=num_switches)
			length += delta_length
			points.extend(generated)
		#Now we recurse!
		if depth>0:
			(generated, cur_point, length) = generate_path_from_point_to_point(cur_point,end_point,calculate_tangent_angle(points),depth-1,
											hairpined=hairpined or not make_cturn,many_hairpins=many_hairpins,
											fuzz_radius=fuzz_radius)
			length += delta_length
			points.extend(generated)
		


	return (points,points[-1],length)

def generate_hairpin_turn(start_point,radius,tangent_in,switchback_num=None,turn_left=None,wobbliness=None,straight_size=None,circle_size=None,uniform_circles=True):
	cur_point = start_point
	cur_tangent = tangent_in
	length = 0
	start_left_num = 0 if turn_left else 1

	#A hairpin has a few choices:
	#	How many switchbacks
	#	Direction of first switchback
	#	"Wobbliness" (circle_percent)
	#	Size of straightways
	turn_left = uniform(0,1)<0.5 									if turn_left == None 		else turn_left
	switchback_num = 2*randrange(TrackGenerator.MIN_HAIRPIN_NUM,TrackGenerator.MAX_HAIRPIN_NUM)	if switchback_num == None	else switchback_num
	wobbliness = uniform(0.45,0.55)									if wobbliness == None		else wobbliness
	straight_size = uniform(TrackGenerator.MIN_STRAIGHT,TrackGenerator.MAX_STRAIGHT)		if straight_size == None	else straight_size
	circle_size = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)	if circle_size == None		else circle_size

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
	last_normal = None
	for a in range(0,switchback_num):
		#Switchback starts with a circle, then a line
		#then we repeat
	
		circle_size = circle_size if uniform_circles else uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)

		#cturn
		(generated, cur_point,delta_length,last_normal) = generate_constant_turn(cur_point,circle_size,cur_tangent,
										circle_percent=wobbliness,turn_left=turn_left,
										turn_against_normal=last_normal)
		length += delta_length
		points.extend(generated)
		cur_tangent = calculate_tangent_angle(points)

		#straight
		(generated, cur_point,delta_length) = generate_straight(cur_point,straight_size,cur_tangent)
		length += delta_length
		points.extend(generated)
		cur_tangent = calculate_tangent_angle(points)


	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length)


def generate_constant_turn_until_facing_point(start_point,radius,tangent_in,goal_point,turn_left=None,turn_against_normal=None,recursed=False):

	#Just a heads up, this section is performance-critical.
	turn_left = uniform(0,1) < 0.5 if turn_left == None else turn_left

	#Calculate preliminary values
	s = start_point
	r = radius
	normal = (-math.sin(tangent_in),math.cos(tangent_in))
	n = normal if not turn_left else (-normal[0],-normal[1])
	g = goal_point
	rn = (r*n[0],r*n[1])
	c = (s[0]+rn[0],s[1]+rn[1])
	gc = (g[0]-c[0],g[1]-c[1])
	sc = (s[0]-c[0],s[1]-c[1])
	t = (math.cos(tangent_in),math.sin(tangent_in))
	x = max(magnitude( gc ),r)

	#Figure out what quadrant we will have to depart the circle at
	#First convert to the tangent-normal basis (TNB reference frame)
	g_prime = (t[0]*gc[0]+t[1]*gc[1],n[0]*gc[0]+n[1]*gc[1])

	#Now find out which quadrant it is in
	quadrant = 1
	if (g_prime[0] >= 0 and -r <= g_prime[1] and g_prime[1] <= 0) or (g_prime[0] >= r and -r <= g_prime[1]):
		quadrant = 1
	elif (g_prime[1] >= 0 and 0 <= g_prime[0] and g_prime[0] <= r) or (g_prime[1] >= r and g_prime[0] <= r):
		quadrant = 2
	elif (g_prime[0] <= 0 and 0 <= g_prime[1] and g_prime[1] <= r) or (g_prime[0] <= -r and g_prime[1] <= r):
		quadrant = 3
	else:
		quadrant = 4

	#For each quadrant, we need to move what is considered the "start point" by 90 degrees
	quadrant_angle = (quadrant-1) * math.pi/2
	cos_a = math.cos(quadrant_angle)
	sin_a = math.sin(quadrant_angle)
	new_s = (cos_a*sc[0]-sin_a*sc[1],sin_a*sc[0]+cos_a*sc[1])

	#This section calculates the angle within a circle quadrant that is needed to turn.
	theta = 0
	if quadrant == 1 or quadrant == 3 or not turn_left:
		inner_angle = math.acos(r/x)
		cross = gc[0]*new_s[1]-gc[1]*new_s[0]
		dot   = gc[0]*new_s[0]+gc[1]*new_s[1]
		outer_angle = math.atan2(cross,dot)
		theta = outer_angle-inner_angle+quadrant_angle
	else:
		#To be honest, I do not know why the dot product should be negated in this case
		#Mathematically I can't find a reason, so I must have just made sign errors somewhere
		#in this implementation.  But to be quite frank, I've spent 3 days of my life trying
		#to get this darn function to work so I'm leaving it like this for now.
		#Fresh eyes would be beneficial, whether it be someone else or future-me.
		inner_angle = math.acos(r/x)
		cross = gc[0]*new_s[1]-gc[1]*new_s[0]
		dot   = gc[0]*new_s[0]+gc[1]*new_s[1]
		outer_angle = math.atan2(cross,-dot)
		theta = outer_angle-inner_angle+quadrant_angle

	#The problem with heading directly to the goal with perfect precision is that once well-positioned,
	#this curve is on a "hair line" of turning the wrong way, leading to a 50% chance of a full turn
	#So if we're very close to a full turn we'll just smudge it.
	if theta > 2*(31/32)*math.pi and not recursed: 
		return generate_constant_turn_until_facing_point(start_point,
								radius,
								tangent_in,
								goal_point,
								turn_left=turn_left,
								turn_against_normal=turn_against_normal,
								recursed=True)
	cp = theta/(math.pi*2)

	toReturn = generate_constant_turn(start_point,radius,tangent_in,turn_left = turn_left, circle_percent = cp)
	return toReturn
	


def get_parametric_circle(start_point,center_point,delta_angle):
	#We can calculate points on a circle using a rotation matrix
	#R(a)*[S-C]+C gives us any point on a circle starting at S with center C with counterclockwise angular distance 'a'
	def output(s,c,a):
		(sx,sy) = s
		(cx,cy) = c
		cos_a    = math.cos(a)
		sin_a    = math.sin(a)
		del_x    = sx-cx
		del_y    = sy-cy
		result_x = cos_a*del_x-sin_a*del_y+cx
		result_y = sin_a*del_x+cos_a*del_y+cy
		return (result_x,result_y)
	return lambda t: output(start_point,center_point,t*delta_angle)


def generate_constant_turn(start_point,radius,tangent_in,turn_left=None,circle_percent=None,turn_against_normal=None):
	if radius > TrackGenerator.MAX_CONSTANT_TURN: rospy.logerr("ERR LARGE RADIUS: " + str(radius))
	#cturns have choices - turn left or turn right
	#Then, they can choose what percent of the circle do they want to turn?
	turn_left = uniform(0,1)<0.5 		if turn_left == None 		else turn_left
	circle_percent = uniform(0.1,0.5)	if circle_percent == None 	else circle_percent

	tangent_vec = (math.cos(tangent_in), math.sin(tangent_in))
	normal_vec  = (-math.sin(tangent_in),math.cos(tangent_in))
	if turn_left: 
		normal_vec = (-normal_vec[0],-normal_vec[1])
	if turn_against_normal != None:
		normal_vec = normalize_vec(turn_against_normal)
	center = (start_point[0]+normal_vec[0]*radius,start_point[1]+normal_vec[1]*radius)
	turn_angle = circle_percent*math.pi*2
	cross_product = np.cross( [tangent_vec[0],tangent_vec[1]],[normal_vec[0],normal_vec[1]] )
	flipper = -1 if cross_product < 0 else 1 #calculates if turning clockwise (-1) or anticlockwise (1)
	circle_function = get_parametric_circle(start_point,center,flipper*turn_angle)

	#Length of circle is, fortunately, easy!  It's simply radius*angle
	length = turn_angle*radius

	fidelity = 360
	points = [circle_function(t/fidelity) for t in range(0,int(fidelity)+1)]

	#Now we want to find the normal vector, because it's useful to have to determine whether it curves inwards or outwards
	#Normal vectors are always parallel to the vector from center to end point
	normal = (points[-1][0]-center[0],points[-1][1]-center[1])

	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length,normal)
	
	

def generate_straight(start_point,length,angle):
	(startx,starty) = start_point

	#Now create a parameterized function in terms of the angle
	#This is easy - tan(angle) = slope, so y = mt + starty, x = t + startx
	#The length of this line is del_x^2+del_y^2 = length^2, so (mt)^2+t^2 = length^2
	#implying t^2 = length^2/(1+m^2)
	#So t ranges from 0 to length/sqrt(1+m^2)
	slope = math.tan(angle)
	tmax = length/math.sqrt(1+slope*slope)

	if angle*slope < 0:
		#I don't actually know the geometrical reason why this is needed :/
		#But if you don't do this, sometimes the line points the wrong way!
		tmax *=-1

	#Since we draw the track by placing lines, we only need the end_points of this!
	#(For other curves we approximate by a bunch of small lines, so we'd need full data)
	#However we actually don't want that because it will mess with the self-intersection-detection
	#later on.
	scale_factor = 10.0
	if tmax >= 0:
		points = [(t/scale_factor+startx,slope*t/scale_factor+starty) for t in range(0,int(scale_factor*math.ceil(tmax)))]
	else:
		points = [(-t/scale_factor+startx,-slope*t/scale_factor+starty) for t in range(0,int(scale_factor*math.ceil(-tmax)))]
	#points = [start_point,(tmax+startx,slope*tmax+starty)]


	#Returns a list of points and the new edge of the racetrack and the change in length
	return (points,points[-1],length)


def convert_points_to_all_positive(xys):
	#If the track dips to the negative side of the x or y axes, shift everything over
	#Returns shifted points tupled with the range over which the points span
	#We also want everything converted to an integer!
	max_neg_x = 0
	max_neg_y = 0
	max_x    = 0
	max_y    = 0

	for point in xys:
		(x,y) = point
		max_neg_x = min(x,max_neg_x)
		max_neg_y = min(y,max_neg_y)
		max_x    = max(x,max_x)
		max_y    = max(y,max_y)

	new_xys = []
	padding = 10
	for point in xys:
		(x,y) = point
		new_xys.append(((x-max_neg_x)+padding,(y-max_neg_y)+padding))

	return (new_xys,int(max_x-max_neg_x)+2*padding,int(max_y-max_neg_y)+2*padding)


def compactify_points(points):
	#Given a list of int points, if any two adjacent points are the same then remove one of them
	remove_list = []
	prev_point = (-10000,-10000)
	def make_int(tup):
		return (int(tup[0]),int(tup[1]))
	for a in range(0,len(points)):
		if (make_int(points[a]) == make_int(prev_point)):
			remove_list.append(a)
		prev_point = points[a]
	for index in sorted(remove_list,reverse=True):
		del points[index]
	return points

def check_if_overlap(points):
	#Naive check to see if track overlaps itself - we remove duplicates from the list and check if size changes
	#(Won't catch overlaps due to track width, only if track center overlaps)
	points = points[:-10] #remove end points as in theory that should also be the start point
	#(I remove extra to be a little generous to it as a courtesy - I don't really care how well the
	#start loops to the end yet)

	#We want to add in the diagonally-connected points, otherwise you can imagine
	#that two tracks moving diagonally opposite could cross eachother inbetween the pixels,
	#fooling our test.
	for index in range(1,len(points)):
		(sx,sy) = points[index-1]
		(ex,ey) = points[index]
		manhattan_distance = abs(ex-sx)+abs(ey-sy)
		if (manhattan_distance > 1):
			#moved diagonally, insert an extra point for it at the end!
			points.append( (sx+1,sy) if ex > sx else (sx-1,sy) )

	return len(set(points)) != len(points)



