import math
from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform
import rospy
from scipy.special import binom
from LauncherUtilities import *
import numpy as np

class GenerationFailedException(Exception):
   """Raised when generator takes too long."""
   pass

class TrackGenerator:

        # Modes
        CIRCLE_AND_LINE_MODE = "Circle&Line"
        BEZIER_MODE          = "Bezier"

        # The rules for autocross:
        #        Straights:                     <=80 meters
        #        Constant Turns:                <=25 meter radius
        #        Hairpin Turns:                 >=4.5 meter outside radius
        #        Slalom:                 Cones in a straight line with 7.5 to 12 meter spacing
        #                                [NOTE: can't be generated at this point, added later]
        #        Track Width:                  >=3 meters
        #        Track Length:                 <=1500 meters

        MIN_STRAIGHT = 20
        MAX_STRAIGHT = 80
        MIN_CONSTANT_TURN = 10
        MAX_CONSTANT_TURN = 25
        MIN_HAIRPIN = 4.5
        MAX_HAIRPIN = 10
        MAX_TRACK_LENGTH = 1500
        LAX_GENERATION = False
        TRACK_MODE = CIRCLE_AND_LINE_MODE
        MAX_HAIRPIN_PAIRS = 1
        MIN_HAIRPIN_PAIRS = 0

        # Debugging parameter to print out when generation fails
        FAILURE_INFO = False

        # The "resolution" of the track, higher is higher resolution
        FIDELITY = 350

        def __init__(self):
                pass

        @staticmethod
        def get_presets():
                """
                Returns a dictionary of generator presets.

                Presets contain all the information that parameterizes track generation.
                Also couples the presets with colloquial names for them.
                """
                return {
                        "Contest Rules": {
                                "MIN_STRAIGHT":10,
                                "MAX_STRAIGHT":80,
                                "MIN_CONSTANT_TURN":10,
                                "MAX_CONSTANT_TURN":25,
                                "MIN_HAIRPIN":4.5,
                                "MAX_HAIRPIN":10,
                                "MAX_HAIRPIN_PAIRS":3,
                                "MAX_LENGTH":1500,
                                "LAX_GENERATION":False,
                                "MODE":TrackGenerator.CIRCLE_AND_LINE_MODE
                        },
                        "Small Straights": {
                                "MIN_STRAIGHT":5,
                                "MAX_STRAIGHT":40,
                                "MIN_CONSTANT_TURN":10,
                                "MAX_CONSTANT_TURN":25,
                                "MIN_HAIRPIN":4.5,
                                "MAX_HAIRPIN":10,
                                "MAX_HAIRPIN_PAIRS":3,
                                "MAX_LENGTH":700,
                                "LAX_GENERATION":True,
                                "MODE":TrackGenerator.CIRCLE_AND_LINE_MODE
                        },
                        "Computer Friendly": {
                                "MIN_STRAIGHT":10,
                                "MAX_STRAIGHT":80,
                                "MIN_CONSTANT_TURN":5,
                                "MAX_CONSTANT_TURN":15,
                                "MIN_HAIRPIN":4.5,
                                "MAX_HAIRPIN":10,
                                "MAX_HAIRPIN_PAIRS":3,
                                "MAX_LENGTH":500,
                                "LAX_GENERATION":True,
                                "MODE":TrackGenerator.CIRCLE_AND_LINE_MODE
                        },
                        "Bezier": {
                                "MIN_STRAIGHT":10,
                                "MAX_STRAIGHT":80,
                                "MIN_CONSTANT_TURN":5,
                                "MAX_CONSTANT_TURN":15,
                                "MIN_HAIRPIN":4.5,
                                "MAX_HAIRPIN":10,
                                "MAX_HAIRPIN_PAIRS":3,
                                "MAX_LENGTH":500,
                                "LAX_GENERATION":True,
                                "MODE":TrackGenerator.BEZIER_MODE
                        }
                }

        @staticmethod
        def get_default_mode_string():
                """Returns the name of the default generation mode"""

                return TrackGenerator.CIRCLE_AND_LINE_MODE

        @staticmethod
        def get_default_preset():
                """Returns the name of the default generation preset."""

                return "Small Straights"

        @staticmethod
        def get_preset_names():
                """Returns a list of all generation preset names."""

                to_return = []
                all_presets = TrackGenerator.get_presets()
                for a in all_presets:
                        to_return.append(a)
                return to_return

        @staticmethod
        def get_preset(name):
                """
                Given a name, returns a dictionary of the preset's values.

                If such a name does not exist, it will send console warnings
                and default to the default preset.
                """

                all_presets = TrackGenerator.get_presets()
                if name in all_presets:
                        return all_presets[name]
                rospy.logerr("No such preset: " + name)
                rospy.logerr("Defaulting to " + get_default_preset())
                return get_preset(get_default_preset())

        @staticmethod
        def set_data(values):
                """
                Sets up TrackGenerator to use the generation parameters 
                specified in the input.
                """

                TrackGenerator.MIN_STRAIGHT = values["MIN_STRAIGHT"]
                TrackGenerator.MAX_STRAIGHT = values["MAX_STRAIGHT"]
                TrackGenerator.MIN_CONSTANT_TURN = values["MIN_CONSTANT_TURN"]
                TrackGenerator.MAX_CONSTANT_TURN = values["MAX_CONSTANT_TURN"]
                TrackGenerator.MIN_HAIRPIN = values["MIN_HAIRPIN"]
                TrackGenerator.MAX_HAIRPIN = values["MAX_HAIRPIN"]
                TrackGenerator.MAX_HAIRPIN_PAIRS = values["MAX_HAIRPIN_PAIRS"]
                TrackGenerator.MIN_HAIRPIN_PAIRS = 1 if TrackGenerator.MAX_HAIRPIN_PAIRS > 0 else 0
                TrackGenerator.MAX_TRACK_LENGTH = values["MAX_LENGTH"]
                TrackGenerator.LAX_GENERATION = values["LAX_GENERATION"]
                TrackGenerator.TRACK_MODE = values["MODE"]

        @staticmethod
        def generate(values):
                """
                Generates the track as pure data 
                  (a List[Tuple[float,float]] list of points on the track)

                Returns a list of points to define the path of the track, 
                  along with a bounding width & height for how big the track is.

                Input is a list of track parameters
                """

                TrackGenerator.set_data(values)
                xys = []
                overlapped = False
                if TrackGenerator.TRACK_MODE == TrackGenerator.CIRCLE_AND_LINE_MODE:
                        generate_function = generate_autocross_trackdrive_track
                else:
                        generate_function = generate_bezier_track
                failure_count = 0
                while overlapped or xys==[]:
                        # Re-generate if the track overlaps itself
                        (xys,twidth,theight) = generate_function((0,0))
                        xys2 = [(int(x[0]),int(x[1])) for x in xys]
                        xys2 = compactify_points(xys2)
                        overlapped = check_if_overlap(xys2)
                        if overlapped:
                                if TrackGenerator.FAILURE_INFO: 
                                        rospy.logerr("Overlap check "+str(failure_count)+" failed")
                                failure_count+=1
                        if failure_count > 10000:
                                raise GenerationFailedException
                
                # Now let's randomly flip it a bit to spice it up
                if uniform(0,1) < 0.5:#flip xys by x
                        xys = [(-x+twidth,y) for (x,y) in xys]
                if uniform(0,1) < 0.5:#flip xys by y
                        xys = [(x,-y+theight) for (x,y) in xys]

                return (xys,twidth,theight)


####################
# Macro Generators #
####################

def generate_bezier_track(start_point):
        """
        In this function we handle the quick&dirty Bezier generator

        This is the entry point for the generator's Bezier mode after generate() is called.
        It takes in an initial point, and returns track data such that 
        we have a list of points
        outlining a closed G1-continuous loop made out of purely Bezier curves.
        """
        xys = []
        total_length = 0

        # Set up goal points
        goal_points = [        
                (start_point[0] + TrackGenerator.MAX_TRACK_LENGTH * 0.08,
                 start_point[1]),
                (start_point[0] + TrackGenerator.MAX_TRACK_LENGTH * 0.12,
                 start_point[1] + TrackGenerator.MAX_TRACK_LENGTH * 0.08),
                (start_point[0] - TrackGenerator.MAX_TRACK_LENGTH * 0.03,
                 start_point[1] + TrackGenerator.MAX_TRACK_LENGTH * 0.12)
        ]

        # Set up info for next component
        initial_tangent = get_random_unit_vector()
        tangent_in = initial_tangent
        normal_in  = get_normal_vector(tangent_in)
        point_in   = start_point
        tangent_out = get_random_unit_vector()

        for point_out in goal_points:
                # Draw next component
	        points_out, tangent_out, normal_out, added_length = connector_bezier(
	                point_in,
	                point_out,
	                tangent_in,
                        tangent_out,
	                normal_in
	        )

                # Set up info for next component
                xys.extend(points_out)
                point_in      = points_out[-1]
                tangent_in    = tangent_out
                normal_in     = normal_out
                total_length += added_length
                tangent_out   = get_random_unit_vector()

        # Set up info for final component
        point_out   = start_point
        tangent_out = initial_tangent

        # Draw final component
        points_out, tangent_out, normal_out, added_length = connector_bezier(
                point_in,
                point_out,
                tangent_in,
                tangent_out,
                normal_in
        )
        xys.extend(points_out)

        return convert_points_to_all_positive(xys)



def generate_autocross_trackdrive_track(start_point):
        """
        In this function we handle the traditional Circle&Line generator

        Takes in an initial point, and returns a G1-continuous closed loop
        made by piecing circles and lines together.
        """
        xys = []
        total_length = 0

        # We start with a small straight
        initial_tangent = get_random_unit_vector(0,math.pi/8)
        tangent_in = initial_tangent
        point_in = start_point
        normal_in = get_normal_vector(tangent_in)
        (points_out, tangent_out, normal_out, added_length) = generic_straight(
                point_in,
                tangent_in,
                normal_in
        )
        total_length += added_length
        xys.extend(points_out)

        # Now we want to set checkpoints to pass through:
        goal_points = [(start_point[0] + TrackGenerator.MAX_TRACK_LENGTH * 0.08,
                        start_point[1]),
                       (start_point[0] + TrackGenerator.MAX_TRACK_LENGTH * 0.12,
                        start_point[1] + TrackGenerator.MAX_TRACK_LENGTH * 0.08),
                       (start_point[0] - TrackGenerator.MAX_TRACK_LENGTH * 0.03,
                        start_point[1] + TrackGenerator.MAX_TRACK_LENGTH * 0.12)]

        # This controls how much it tries to salvage a bad run
        # It turns out that most times it fails, its not salvageable,
        # so I set it to 1 so that as soon as it fails it scraps the run.
        max_fails = 1
        fails = 0

        # And now we generate towards each goal point
        for goal_point in goal_points:
                prev_xys = xys
                
                # Prepare inputs
                tangent_in = tangent_out
                normal_in  = normal_out
                point_out  = goal_point
                point_in   = points_out[-1]

                # Generate from point to point
                (points_out, tangent_out, normal_out, added_length) = generate_point_to_point(
                        point_in,
                        point_out,
                        tangent_in,
                        normal_in,
                        20
                )
                total_length += added_length
                xys.extend(points_out)

                # Now let's do early-checking for overlaps
                test = compactify_points([(int(x[0]), int(x[1])) for x in xys])
                if check_if_overlap(test): 
                        if TrackGenerator.FAILURE_INFO: 
                                rospy.logerr("Early Overlap Checking: Failed")

                        # Generation failed test, undo last bit
                        fails += 1
                        total_length -= added_length
                        xys = prev_xys

                        if fails == max_fails:
                                return (test, 0, 0)

        # Now lets head back to the start:
        # We're gonna set a checkpoint that is close but not exactly the start point
        # so that we have breathing room for the final manouever:
        directing_point = (start_point[0] - TrackGenerator.MAX_STRAIGHT * 0.5,
                           start_point[1] + TrackGenerator.MAX_CONSTANT_TURN * 2)

        # Prepare inputs
        tangent_in = tangent_out
        normal_in  = normal_out
        point_out  = directing_point
        point_in   = points_out[-1]

        # Generate from point to point
        (points_out, tangent_out, normal_out, added_length) = generate_point_to_point(
                point_in,
                point_out,
                tangent_in,
                normal_in,
                0
        )
        total_length += added_length
        xys.extend(points_out)

        # Now we will add a circle to point directly away from start point
        tangent_in  = tangent_out
        normal_in   = normal_out
        point_in    = points_out[-1]
        tangent_out = scale_vector(initial_tangent,-1)

        # We need to calculate the turn angle (angle between 2 vectors):
        outer_turn_angle = math.acos(
                - tangent_in[0] * tangent_out[0]
                - tangent_in[1] * tangent_out[1]
        )
        circle_turn_angle = math.pi - outer_turn_angle
        circle_turn_percent = circle_turn_angle / (2 * math.pi)
        circle_radius = uniform(
                TrackGenerator.MIN_CONSTANT_TURN,
                TrackGenerator.MAX_CONSTANT_TURN
        )

        # Now we draw this circle:
        (points_out, tangent_out, normal_out, added_length) = generic_constant_turn(
                point_in,
                tangent_in,
                normal_in,
                params={
                        "turn_against_normal": False,
                        "circle_percent":      circle_turn_percent,
                        "radius":              circle_radius
                }
        )
        total_length += added_length
        xys.extend(points_out)

        # And now we add a half-turn to point us directly back to the start.
        # Radius is calculated by finding distance when projected along the normal
        tangent_in  = tangent_out
        normal_in   = normal_out
        point_in    = points_out[-1]
        tangent_out = initial_tangent
        diff = subtract_vectors(point_in,start_point)
        circle_radius = (diff[0] * normal_in[0] + diff[1] * normal_in[1])/2

        # Now we draw the circle:
        (points_out, tangent_out, normal_out, added_length) = generic_constant_turn(
                point_in,
                tangent_in,
                normal_in,
                params={
                        "turn_against_normal": False,
                        "circle_percent":      0.5,
                        "radius":              abs(circle_radius)
                }
        )
        total_length += added_length
        xys.extend(points_out)

        # And then add a straight to connect back to the start
        tangent_in = tangent_out
        normal_in  = normal_out
        point_in   = points_out[-1]
        straight_length = get_distance(point_in, start_point) * 1.1
        (points_out, tangent_out, normal_out, added_length) = generic_straight(
                point_in,
                tangent_in,
                normal_in,
                params = {
                        "length": straight_length
                }
        )
        total_length += added_length
        xys.extend(points_out)


        # Sometimes the tangents don't actually match up
        # so if that happens, we throw out the track and start anew.
        if get_distance(initial_tangent,tangent_out) > 0.01:
                return generate_autocross_trackdrive_track(start_point)


        if not TrackGenerator.LAX_GENERATION:
                # Check if accidentally created too big of a straight at the very end
                if (straight_length + TrackGenerator.MIN_STRAIGHT 
                 > TrackGenerator.MAX_STRAIGHT):

                        # We always start each track with a minimum-length straight, 
                        # which is joined up with the final straight,
                        # hence the addition of MIN_STRAIGHT here.
                        return generate_autocross_trackdrive_track(start_point)

                # Check if whole track is too big
                elif total_length > TrackGenerator.MAX_TRACK_LENGTH:
                        return generate_autocross_trackdrive_track(start_point)

        return convert_points_to_all_positive(xys)


def generate_point_to_point(point_in,
                            point_out,
                            tangent_in,
                            normal_in,
                            fuzz_radius,
                            depth = 20,
                            hairpined = False,
                            many_hairpins = False):
        """
        Generates a track from point_in to point_out with incoming tangent tangent_in,
        and incoming normal normal_in.

        Does not necessarily exactly end at point_out, it merely aims to get within fuzz_radius.

        depth is the max amout of times it may recurse (this is a recursive function that
        incrementally builds the track up recursion by recursion).

        hairpined, when True, signals that a previous recursion of this function placed a hairpin.
        many_hairpins, when False, disallows multiple hairpins.  So if hairpined is True then
        many_hairpins being False would prevent additional hairpins.

        Outputs (points_out, tangent_out, normal_out, added_length)
        """

        added_length = 0
        xys = []
        
        # We start out by refocusing ourselves towards point_out, using a refocus_constant_turn.
        (points_out, tangent_out, normal_out, delta_length) = (
                refocus_constant_turn(
                        point_in,
                        point_out,
                        tangent_in,
                        normal_in,
                )
        )
        added_length += delta_length
        xys.extend(points_out)

        # Prepare data for next component
        point_in   = points_out[-1]
        tangent_in = tangent_out
        normal_in  = normal_out

        # Now we want to know how close we are to the goal - if we're close enough
        # to just directly draw a straight path to it.
        distance_from_end = get_distance(point_in,point_out)
        if distance_from_end <= TrackGenerator.MAX_STRAIGHT + fuzz_radius:
                straight_length = min(distance_from_end, TrackGenerator.MAX_STRAIGHT)
                (points_out, tangent_out, normal_out, delta_length) = generic_straight(
                        point_in,
                        tangent_in,
                        normal_in,
                        params = {"length":straight_length}
                )
                added_length += delta_length
                xys.extend(points_out)

        # If we are far away from the goal, we want to get closer to it in a
        # "random" way to create variance in the track.
        else:
                # We'll start by just going as far as we can to the goal.
                # Unless we're very close, then we only go half as far to
                # give us some space (and add variance)
                if distance_from_end <= (TrackGenerator.MAX_STRAIGHT * 1.2):
                        straight_length = TrackGenerator.MAX_STRAIGHT
                else:
                        straight_length = TrackGenerator.MAX_STRAIGHT/2
                (points_out, tangent_out, normal_out, delta_length) = generic_straight(
                        point_in,
                        tangent_in,
                        normal_in,
                        params = {"length":straight_length}
                )
                added_length += delta_length
                xys.extend(points_out)

                # Prepare data for next component
                point_in   = points_out[-1]
                tangent_in = tangent_out
                normal_in  = normal_out
                

                # Now we add some variance by choosing between cturns or hairpins
                # There is no special significance to the 70% value, it happens to
                # get a good balance.
                make_cturn = uniform(0, 1) < 0.7

                # But of course, we can't draw a hairpin if we've drawn too many already.
                if make_cturn or (hairpined and not many_hairpins):
                        (points_out, tangent_out, normal_out, delta_length) = (
                                generic_constant_turn(
                                        point_in,
                                        tangent_in,
                                        normal_in,
                                        params = {
                                                "circle_percent":uniform(0.1, 0.3)
                                        }
                                )
                        )
                        added_length += delta_length
                        xys.extend(points_out)

                        # Prepare data for next component
                        point_in   = points_out[-1]
                        tangent_in = tangent_out
                        normal_in  = normal_out

                        # And now let's add another circle turning the other way
                        # so that we make the track more "winding", and allow us to
                        # make real progress towards the goal.
                        (points_out, tangent_out, normal_out, delta_length) = (
                                generic_constant_turn(
                                        point_in,
                                        tangent_in,
                                        normal_in,
                                        params = {
                                                "circle_percent":uniform(0.1, 0.3)
                                        }
                                )
                        )
                        added_length += delta_length
                        xys.extend(points_out)

                        # Prepare data for next component
                        point_in   = points_out[-1]
                        tangent_in = tangent_out
                        normal_in  = normal_out

                # Make a hairpin
                else:
                        # We only want an even amount of turns so that we can leave 
                        # heading the same direction we entered.
                        # Otherwise due to the way we head towards the path, 
                        # its basically guaranteed we get a self-intersection.
                        num_switches = 2*randrange(
                                TrackGenerator.MIN_HAIRPIN_PAIRS,
                                TrackGenerator.MAX_HAIRPIN_PAIRS
                        )
                        (points_out, tangent_out, normal_out, delta_length) = (
                                generic_hairpin_turn(
                                        point_in,
                                        tangent_in,
                                        normal_in,
                                        params = {
                                                "switchbacks":num_switches
                                        }
                                )
                        )
                        added_length += delta_length
                        xys.extend(points_out)

                        # Prepare data for next component
                        point_in   = points_out[-1]
                        tangent_in = tangent_out
                        normal_in  = normal_out

                # Now, we recurse
                if depth > 0:
                        (points_out, tangent_out, normal_out, added_length) = (
                                generate_point_to_point(
                                        point_in,
                                        point_out,
                                        tangent_in,
                                        normal_in,
                                        fuzz_radius,
                                        depth = depth - 1,
                                        hairpined = hairpined or not make_cturn,
                                        many_hairpins = many_hairpins
                                )
                        )
                        added_length += delta_length
                        xys.extend(points_out)


        return (xys, tangent_out, normal_out, added_length)


####################
# Micro Generators #
####################
#
# There are four types of micro generators:
#
# 1: "generic"
#         These have the signature generic_<type>(point_in,tangent_in,normal_in,params)
#         and output (points_out, tangent_out, normal_out, added_length)
#
#         Generics just create a generically random component, with no extra requirements.
#
# 2: "refocus"
#         These have the signature refocus_<type>(point_in,point_out,tangent_in,normal_in,params)
#         with the same output as generics.
#
#         Refocusers ensure that the outgoing tangent points towards point_out
#
# 3: "connector"
#         These have the signature:
#                 connector_<type>(point_in,point_out,tangent_in,tangent_out,normal_in,params)
#         with the same output as generics
#
#         Connectors ensure that the component ends at point_out, with outgoing tangent tangent_out
#
# 4: "parametric"
#         These have variable signatures, but always output a function of one variable
#         which rangens from 0 to 1, parametrizing the curve they represent in terms of it.
#
#########################################################

def de_parameterize(func):
        """Given a parametric function, turn it into a list of points"""
        return [func(1.0 * t / (TrackGenerator.FIDELITY - 1)) 
                for t in range(0 , TrackGenerator.FIDELITY)]



def generic_straight(point_in,
                     tangent_in,
                     normal_in,
                     params = {}):
        """
        Creates a straight line micro generator

        params["length"]: specifies how long to make the straight
        """

        # Load in params
        if "length" in params:
                length = params["length"]
        else:
                length = uniform(TrackGenerator.MIN_STRAIGHT,TrackGenerator.MAX_STRAIGHT)

        # Prepare outputs
        straight_func = parametric_straight(tangent_in,point_in,length)
        tangent_out   = tangent_in
        normal_out    = normal_in
        added_length  = length

        return (
                de_parameterize(straight_func),
                tangent_out,
                normal_out,
                added_length
        )



def generic_constant_turn(point_in,
                          tangent_in,
                          normal_in,
                          params = {}):
        """
        Creates a circle segment micro generator.

        params["turn_against_normal"]: boolean, when true then the 
                                       circle will have the opposite normal.
        params["radius"]: radius of the circle
        params["circle_percent"]: percent of a full turn the circle should undergo
        """

        # Load in params
        if "turn_against_normal" in params:
                turn_against_normal = params["turn_against_normal"]
        else:
                turn_against_normal = uniform(0,1) < 0.5

        if turn_against_normal:
                # We need to flip our reference frame.
                normal_in = scale_vector(normal_in, -1)

        if "radius" in params:
                radius = params["radius"]
        else: 
                radius = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)

        if "circle_percent" in params:
                circle_percent = params["circle_percent"]
        else: 
                # Note: if circles turn too much, risk of self-intersection rises dramatically.
                circle_percent = uniform(0.1,0.5)

        center = add_vectors(point_in, scale_vector(normal_in,radius))

        # We need to calculate which direction to draw the circle, too
        # Traditionally it is drawn counter-clockwise, but it would need to go
        # clockwise if the "handedness" of the system is opposite normal.
        # We can calculate handedness through the sign of the determinant
        # of the tangent and normal matrices
        def sgn(x):
                """-1 if x is negative, +1 if positive, 0 if 0."""
                return -1 if x < 0 else 1 if x > 0 else 0
        handedness = sgn(tangent_in[0] * normal_in[1] - tangent_in[1] * normal_in[0])
        turn_angle = handedness * circle_percent * math.pi * 2

        # And now we can grab output points
        circle_function = parametric_circle(point_in, center, turn_angle)
        points_out = de_parameterize(circle_function)

        # Calculate total length
        added_length = turn_angle * radius

        # Now we want to find the new normal vector, 
        normal_out = normalize_vec((points_out[-1][0] - center[0], points_out[-1][1] - center[1]))

        # And finally recalculate the tangent:
        tangent_out = calculate_tangent_vector(points_out)

        # Returns a list of points and the new edge of the racetrack and the change in length
        return (points_out,tangent_out,normal_out,added_length)
        

def generic_hairpin_turn(point_in,
                         tangent_in,
                         normal_in,
                         params = {}):
        """
        Creates a hairpin micro generator.

        params["turn_against_normal"]: boolean, when true then the 
                                       hairpin will have the opposite normal.

        params["radius"]: radius of the circle

        params["uniform_circles"]: if True, all circles have the same radius (params["radius"])
                                   if False, all circles have random radii to add variety.

        params["straight_length"]: the length of the straight segments

        params["wobbliness"]: The percentage of circumference of the circles
                              This will be overridden if the function detects that
                              The hairpin would self-intersect with the given wobbliness.

        params["switchbacks"]: The amount of turns in the hairpin
        """

        xys = []
        added_length = 0

        # Load in params
        if "turn_against_normal" in params:
                turn_against_normal = params["turn_against_normal"]
        else:
                turn_against_normal = uniform(0,1)<0.5

        if turn_against_normal:
                # We need to flip our reference frame.
                normal_in = scale_vector(normal_in, -1)

        if "radius" in params:
                radius = params["radius"]
        else: 
                radius = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)

        if "uniform_circles" in params:
                uniform_circles = params["uniform_circles"]
        else: 
                uniform_circles = uniform(0, 1) < 0.5

        if "straight_length" in params:
                straight_length = params["straight_length"]
        else:
                straight_length = uniform(TrackGenerator.MIN_STRAIGHT,TrackGenerator.MAX_STRAIGHT)

        if "wobbliness" in params:
                wobbliness = params["wobbliness"]
        else:
                wobbliness = uniform(0.45, 0.55)

        if "switchbacks" in params:
                switchbacks = params["switchbacks"]
        else:
                switchbacks = 2 * randrange(
                        TrackGenerator.MIN_HAIRPIN_PAIRS,
                        TrackGenerator.MAX_HAIRPIN_PAIRS
                )

        # We are interested in making sure switchbacks never intersect
        # If you draw a diagram, this gives you a diamond with angles pi/2,pi/2,L, and pi/2-L 
        # where L is:
        # ((2*pi*(1-wobbliness))/2)
        # Using trigonometry, we can calculate the radius to intersection
        # in terms of the angle and circle radius:
        # intersect_point = radius * tan(L)
        # If intersect_point is greater than max_intersection, we're good!
        # Otherwise we want to cap it there.  So we find the inverse-function for "wobbliness"
        # 1-atan2(intersect_point,radius)/math.pi = wobbliness
        # max_intersection is the distance we tolerate intersections beyond.
        max_intersection = 50
        angle_l = (2 * math.pi * (1 - wobbliness)) / 2
        intersect_point = radius * math.tan(angle_l)
        if intersect_point < max_intersection:
                wobbliness = 1 - math.atan2(max_intersection, radius) / math.pi

        # Now we loop through each segment of the hairpin
        for a in range(0, switchbacks):
                # Switchback starts with a circle, then a line

                # Get the desired radius of the circle
                if not uniform_circles:
                        radius = uniform(
                                TrackGenerator.MIN_CONSTANT_TURN, 
                                TrackGenerator.MAX_CONSTANT_TURN
                        )

                # Do the circle part
                (points_out, tangent_out, normal_out, delta_length) = generic_constant_turn(
                        point_in,
                        tangent_in, 
                        normal_in,
                        params = {
                                "turn_against_normal": True,
                                "circle_percent":      wobbliness,
                                "radius":              radius
                        }
                )
                added_length += delta_length
                xys.extend(points_out)

                # Prepare for next component
                point_in   = points_out[-1]
                tangent_in = tangent_out
                normal_in  = normal_out

                # Now tack on the straight
                (points_out, tangent_out, normal_out, delta_length) = generic_straight(
                        point_in,
                        tangent_in,
                        normal_in,
                        params = {
                                "length": straight_length
                        }
                )
                added_length += delta_length
                xys.extend(points_out)

                # Prepare for next component
                # For hairpins, normals should always flip!
                # That way they zigzag
                point_in   = points_out[-1]
                tangent_in = tangent_out
                normal_in = scale_vector(normal_out, -1)


        return (xys, tangent_out, normal_out, added_length)

def refocus_constant_turn(point_in,
                          point_out,
                          tangent_in,
                          normal_in,
                          params = {}):
        """
        Creates a circle segment micro generator.

        params["turn_against_normal"]: boolean, when true then the 
                                       circle will have the opposite normal.
        params["radius"]: radius of the circle
        params["recursed"]: if true, prevent this function from recursing again
        """

        # Load in params
        if "turn_against_normal" in params:
                turn_against_normal = params["turn_against_normal"]
        else:
                turn_against_normal = uniform(0,1) < 0.5

        if turn_against_normal:
                # We need to flip our reference frame.
                normal_in = scale_vector(normal_in, -1)

        if "radius" in params:
                radius = params["radius"]
        else: 
                radius = uniform(TrackGenerator.MIN_CONSTANT_TURN,TrackGenerator.MAX_CONSTANT_TURN)

        if "recursed" in params:
                recursed = params["recursed"]
        else:
                recursed = False

        center = add_vectors(point_in, scale_vector(normal_in,radius))

        # We need to calculate which direction to draw the circle, too
        # Traditionally it is drawn counter-clockwise, but it would need to go
        # clockwise if the "handedness" of the system is opposite normal.
        # We can calculate handedness through the sign of the determinant
        # of the tangent and normal matrices
        def sgn(x):
                """-1 if x is negative, +1 if positive, 0 if 0."""
                return -1 if x < 0 else 1 if x > 0 else 0
        handedness = sgn(tangent_in[0] * normal_in[1] - tangent_in[1] * normal_in[0])

        # Now we use a rotation matrix to parameterize intermediate points:
        # Given start S and center C, any point on the circle angle A away is:
        # R_A[S-C] + C
        # Let us box this up in a nice function:
        def intermediate_point(s,c,a):
                (sx,sy) = s
                (cx,cy) = c
                cos_a    = math.cos(a)
                sin_a    = math.sin(a)
                del_x    = sx - cx
                del_y    = sy - cy
                result_x = cos_a * del_x - sin_a * del_y + cx
                result_y = sin_a * del_x + cos_a * del_y + cy
                return (result_x, result_y)
        circle_func = lambda a: intermediate_point(point_in,center,a)

        # We are going to iteratively walk around the circle until we are facing the right way
        points_out = []
        max_range = TrackGenerator.FIDELITY
        step_size = 1.0 / max_range
        angle = 0
        for t in range(0,max_range):
                angle = t * step_size * 2 * math.pi
                points_out.append(
                        circle_func(angle*handedness)
                )
                if t != 0:
                        # Check if we're pointing in the right direction
                        # This equates to checking if end_point lies on the 
                        # line at points[-1] with the appropriate tangent.
                        (sx,sy) = points_out[-1]
                        (ex,ey) = point_out
                        appropriate_angle = calculate_tangent_angle(points_out)
                        if t > 0.8 * max_range and not recursed:
                                # Very big turn, we don't like that!  
                                # We'll just turn the other way instead
                                return refocus_constant_turn(
                                        point_in,
                                        point_out,
                                        tangent_in,
                                        normal_in,
                                        params = {
                                                "recursed":True,
                                                "radius":radius,
                                                "turn_against_normal":True
                                        }
                                )
                        if abs(
                                cap_angle(appropriate_angle) 
                               -cap_angle(math.atan2((ey - sy), (ex - sx)))
                        ) < 0.01:
                                # Would do equality checking but limited precision, 
                                # we just check if close!
                                break

        # Length of circle is, fortunately, easy!  It's simply radius*angle
        added_length = angle*radius

        # Now we want to find the new normal vector, 
        normal_out = normalize_vec((points_out[-1][0] - center[0], points_out[-1][1] - center[1]))

        # And finally recalculate the tangent:
        tangent_out = calculate_tangent_vector(points_out)

        # Returns a list of points and the new edge of the racetrack and the change in length
        return (points_out,tangent_out,normal_out,added_length)



def connector_bezier(point_in,
                     point_out, 
                     tangent_in,
                     tangent_out, 
                     normal_in, 
                     params = {}):
        """
        Creates a cubic bezier micro generator.

        params["scale_in"]:  roughly corresponds to how "straight" the start of the bezier is.
        params["scale_out"]: roughly corresponds to how "straight" the end of the bezier is.

        This function does not accurately keep track of normals and track length, so
        it cannot be used when LAX Generation is off.
        """

        # Load in params
        if "scale_in" in params:
                scale_in = params["scale_in"]
        else:
                scale_in = uniform(10,100)
        if "scale_out" in params:
                scale_out = params["scale_out"]
        else:
                scale_out = uniform(10,100)


        # The incoming tangent is the same as the line from P0 to P1
        # Outgoing tangent is the same as the line from P(n-1) to P(n)
        # Where P0, P1, ..., P(n-1), P(n) are the control points
        # All other control points are free to be selected.
        p0_to_p1 = scale_vector(tangent_in,scale_in)
        p0 = point_in
        p1 = (p0[0] + p0_to_p1[0], p0[1] + p0_to_p1[1])

        pn_1_to_pn = scale_vector(tangent_out,scale_out)
        pn = point_out
        pn_1 = (pn[0] - pn_1_to_pn[0], pn[1] - pn_1_to_pn[1])

        control_points = [p0, p1, pn_1, pn]

        # Prepare outputs
        bez_out = parametric_bezier(control_points)
        normal_out = normal_in
        added_length = 0

        return (
                de_parameterize(bez_out),
                tangent_out,
                normal_out,
                added_length
        )
        


def parametric_bezier(control_points):
        """
        This function will itself return a function of a parameterized bezier
        That is, the result will be a function that takes a time parameter from 0 to 1
        and traveling along it results in the points on the bezier.
        I made this code match the Bezier curve definition on wikipedia as closely as
        possible (Explicit definition, not the recursive one)
        """
        def to_return(cp,t):
                the_sum_x = 0
                the_sum_y = 0
                n = len(cp)
                for i in range(n):
                        coefficient = binom(n-1, i) * (1 - t)**(n - i - 1) * t**i
                        the_sum_x += coefficient * cp[i][0]
                        the_sum_y += coefficient * cp[i][1]
                return (the_sum_x, the_sum_y)
        return lambda t: to_return(control_points, t)



def parametric_straight(slope_vec, start_point, line_length):
        """Returns the parametric function of a line given a slope and start point"""
        def to_return(slope, start, length, t):
                return add_vectors(start, scale_vector(slope, 1.0 * t * line_length))
        return lambda t: to_return(slope_vec,start_point, line_length, t)



def parametric_circle(start_point, center_point, delta_angle):
        """
        Returns a function in terms of t \elem [0,1] to parameterize a circle        

        We can calculate points on a circle using a rotation matrix
        R(a)*[S-C]+C gives us any point on a circle starting at S with center C 
        with counterclockwise angular distance 'a'
        """
        def output(s, c, a):
                (sx, sy) = s
                (cx, cy) = c
                cos_a    = math.cos(a)
                sin_a    = math.sin(a)
                del_x    = sx - cx
                del_y    = sy - cy
                result_x = cos_a * del_x - sin_a * del_y + cx
                result_y = sin_a * del_x + cos_a * del_y + cy
                return (result_x, result_y)
        return lambda t: output(start_point, center_point, t * delta_angle)


