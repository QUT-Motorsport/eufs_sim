import math

def calculate_tangent_angle(xys):
	#Calculate direction of outgoing tangent of a set of points
	#Is an angle!
	sx = xys[-2][0]
	sy = xys[-2][1]
	ex = xys[-1][0]
	ey = xys[-1][1]
	return math.atan2((ey-sy),(ex-sx))

def calculate_tangent_vector(xys):
	#Calculate direction of outgoing tangent of a set of points
	#Is a vector!
	angle = calculate_tangent_angle(xys)
	return (math.cos(angle),math.sin(angle))

def calculate_vector_angle(vec):
	return calculate_tangent_angle([(0,0),vec])

def calculate_vector_from_angle(angle):
	return (math.cos(angle),math.sin(angle))

def cap_angle(ang):
	#Returns angle between 0 and 2*math.pi
	if ang < 0:
		return cap_angle(ang+2*math.pi)
	elif ang >= 2*math.pi:
		return cap_angle(ang-2*math.pi)
	return ang

def cap_angle_odd(ang):
	#Returns angle between -math.pi and math.pi
	ang = cap_angle(ang)
	if ang > math.pi:
		ang = ang-2*math.pi
	return ang

def magnitude(vec):
	(a,b) = vec
	return math.sqrt(a*a+b*b)

def normalize_vec(vec):
	(a,b) = vec
	mag = math.sqrt(a*a+b*b)
	return (a/mag,b/mag)
