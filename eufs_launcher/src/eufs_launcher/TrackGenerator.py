import math
from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform
import rospy
from scipy.special import binom
from LauncherUtilities import *
import numpy as np

"""
################################################################################
################################################################################
################################################################################
###########################Track Generator Overview#############################
################################################################################
################################################################################
################################################################################
###########################Functions For Public Use#############################
################################################################################
#                                                                              #
#        TrackGenerator.generate(values: List[float])                          #
#                        -> Tuple[List[Tuple[float,float]],int,int]            #
#                Takes in a list of preset data.  Check out the get_presets()  #
#                function for a thorough and perpetually up to date list of    #
#                each piece of data.                                           #
#                This returns a tuple of the form (xys,width,height), where:   #
#                        xys:                                                  #
#                                A list of points that outline the             #
#                                generated track.                              #
#                        width:                                                #
#                                The width that is spanned by xys              #
#                        height:                                               #
#                                The height that is spanned by xys             #
#                There will always be a margin of at least 5 on the returned   #
#                xys, so width and height are always at least 10 larger than   #
#                if the range was strictly calculated.                         #
#                This is done for the sake of ConversionTools' TrackImage      #
#                specification, which requires the margin.                     #
#                                                                              #
#        TrackGenerator.get_presets()  -> List[Tuple[str,List[float]]]         #
#                Returns a list of all generator presets (the str in the tuple)#
#                coupled with their preset values (the list in the tuple)      #
#                                                                              #
#        TrackGenerator.get_default_preset() -> str                            #
#                Returns the default preset (usually "Small Straights")        #
#                                                                              #
#        TrackGenerator.get_preset_names() -> List[str]                        #
#                Returns a list of all the generator presets.                  #
#                                                                              #
#        TrackGenerator.get_preset(name: str) -> List[float]                   #
#                Returns a list of all the preset data of the specified        #
#                preset (the one with the matching name).                      #
#                                                                              #
################################################################################
#                                                                              #
#        There are other functions available (many others), but they are not   #
#        necessarily meant for use outside this file.                          #
#                                                                              #
#        That doesn't stop them from being used, of course - but depending on  #
#        the circumstances, their use implies that they maybe should be        #
#        factored out into a seperate library.                                 #
#                                                                              #
#        Since the LauncherModule/TrackGenerator/ConversionTools ecosystem is  #
#        still in a state of flux, function signatures for all functions but   #
#        the ones listed above are subject to change without warning.          #
#                                                                              #
################################################################################
"""

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
        MAX_TRACK_LENGTH = 1500
        LAX_GENERATION = False
        TRACK_MODE = CIRCLE_AND_LINE_MODE

        # Debugging parameter to print out when generation fails
        FAILURE_INFO = False

        # The "resolution" of the track, higher is higher resolution
        FIDELITY = 350

        def __init__(self):
                pass

        @staticmethod
        def get_presets():
                """
                Returns a list of generator presets.

                Presets contain all the information that parameterizes track generation.
                Also couples the presets with colloquial names for them.
                """
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
                """Returns the name of the default generation mode"""

                return TrackGenerator.CIRCLE_AND_LINE_MODE

        @staticmethod
        def get_default_mode_number():
                """Gets default id of the default generation mode."""

                return TrackGenerator.get_number_from_mode(
                        TrackGenerator.get_default_mode_string()
                )

        @staticmethod
        def get_mode_from_number(num):
                """Given a generation mode id, return its name."""

                if     num == 0: return TrackGenerator.CIRCLE_AND_LINE_MODE
                elif   num == 1: return TrackGenerator.BEZIER_MODE
                return get_default_mode_string()

        @staticmethod
        def get_number_from_mode(mode):
                """Given a generation mode name, return its id."""

                if     mode == TrackGenerator.CIRCLE_AND_LINE_MODE: return 0
                elif   mode == TrackGenerator.BEZIER_MODE: return 1
                return get_default_mode_number()

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
                        to_return.append(a[0])
                return to_return

        @staticmethod
        def get_preset(name):
                """
                Given a name, returns a list of the preset's values.

                If such a name does not exist, it will send console warnings
                and default to the default preset.
                """

                all_presets = TrackGenerator.get_presets()
                for a in all_presets:
                        if a[0] == name:
                                return a[1]
                rospy.logerr("No such preset: " + name)
                rospy.logerr("Defaulting to " + get_default_preset())
                return get_preset(get_default_preset())

        @staticmethod
        def set_data(values):
                """
                Sets up TrackGenerator to use the generation parameters 
                specified in the input.
                """

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

        for point_out in goal_points[:-1]:
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

def connector_bezier(point_in,
                     point_out, 
                     tangent_in,
                     tangent_out, 
                     normal_in, 
                     params={}):
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



