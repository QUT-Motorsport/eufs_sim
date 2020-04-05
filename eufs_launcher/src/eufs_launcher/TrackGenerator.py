import math
from PIL import Image
from PIL import ImageDraw
import random
import rospy
from scipy.special import binom
from LauncherUtilities import *
import numpy as np
import collections


class GenerationFailedException(Exception):
        """Raised when generator takes too long."""
        pass


class GeneratorContext:
        """
        Context manager for the generator.

        Initialized with the generator parameters, and optionally a function to be
        called if generation fails.

        Failure can also be accessed by checking generator_context_instance.failed
        if preferred.

        Meant for use in with-blocks.
        """
        def __init__(self, values, failure_function=lambda: None):
                self.values = values
                self.failed = False
                self.failure_function = failure_function

        def __enter__(self):
                TrackGenerator.has_context = True
                TrackGenerator.set_data(self.values)

        def __exit__(self, exc_type, exc_value, traceback):
                TrackGenerator.has_context = False
                if exc_type is GenerationFailedException:
                        rospy.logerr(
                                "\nError!  The generator could not generate in time.\n" +
                                "Maybe try different parameters?\n" +
                                "Turning on Lax Generation and increasing MAX_STRAIGHT" +
                                " and MIN_STRAIGHT usually helps."
                        )
                        self.failed = True
                        self.failure_function()


class Parametrization:
        """
        Class that is used to seemlessly store and combine parametrization functions
        for the generator.

        Parametrization functions are defined as functions which take a number \elem [0,1]
        and return a point.  It is used to define geometric curves in terms of a percentage.
        So if you had a circle parametrization function, and gave it an input 0.25, it would
        return the point 25% of the way around the circle segment.
        """
        def __init__(self, func):
                self._func = func

        def __call__(self, num):
                return self._func(num)

        @staticmethod
        def compose(others):
                """
                Combines two parametrizations in a way such that the composite
                still only takes a number from 0 to 1.
                """

                amount = len(others)
                threshold = 1.0/amount

                def composite(num):
                        # Check out of bounds
                        if num <= 0:
                                return others[0](0)
                        elif num >= 1:
                                return others[-1](1)

                        # Acts as composition of components
                        i = 0
                        while True:
                                if num < threshold:
                                        return others[i](amount * num)
                                else:
                                        num -= threshold
                                        i += 1
                return Parametrization(composite)


class TrackGenerator:
        """
        Handles track generation.
        """

        # Only allow track generator if it is in the right context
        has_context = False

        # Lists for micro generators
        # These will be expanded by micro generators

        double_placement_set = set([])
        generic_micro_list = []
        refocus_micro_list = []
        connector_micro_list = []
        linear_micro_list = []
        special_micro_list = []
        rare_micro_list = []
        start_micro_list = []
        previous_micro = ""

        # The component__weighting_dict stores the weightings for each component
        # And defaults to 0 if the component isn't found.
        component_weighting_dict = collections.defaultdict(lambda: 0)

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
                                "MIN_STRAIGHT": 10,
                                "MAX_STRAIGHT": 80,
                                "MIN_CONSTANT_TURN": 10,
                                "MAX_CONSTANT_TURN": 25,
                                "MIN_HAIRPIN": 4.5,
                                "MAX_HAIRPIN": 10,
                                "MAX_HAIRPIN_PAIRS": 3,
                                "MAX_LENGTH": 1500,
                                "LAX_GENERATION": False,
                                "TRACK_WIDTH": 3.5,
                                "COMPONENTS": {
                                        "STRAIGHT": 1,
                                        "CONSTANT_TURN": 0.7,
                                        "HAIRPIN_TURN": 0.3,
                                }
                        },
                        "Contest Rules Slaloms": {
                                "MIN_STRAIGHT": 10,
                                "MAX_STRAIGHT": 80,
                                "MIN_CONSTANT_TURN": 10,
                                "MAX_CONSTANT_TURN": 25,
                                "MIN_HAIRPIN": 4.5,
                                "MAX_HAIRPIN": 10,
                                "MAX_HAIRPIN_PAIRS": 3,
                                "MAX_LENGTH": 1500,
                                "LAX_GENERATION": False,
                                "TRACK_WIDTH": 3.5,
                                "COMPONENTS": {
                                        "STRAIGHT": 1,
                                        "CONSTANT_TURN": 0.7,
                                        "HAIRPIN_TURN": 0.3,
                                        "SLALOM": 0.3
                                }
                        },
                        "Small Straights": {
                                "MIN_STRAIGHT": 5,
                                "MAX_STRAIGHT": 40,
                                "MIN_CONSTANT_TURN": 10,
                                "MAX_CONSTANT_TURN": 25,
                                "MIN_HAIRPIN": 4.5,
                                "MAX_HAIRPIN": 10,
                                "MAX_HAIRPIN_PAIRS": 3,
                                "MAX_LENGTH": 700,
                                "LAX_GENERATION": True,
                                "TRACK_WIDTH": 3.5,
                                "COMPONENTS": {
                                        "STRAIGHT": 1,
                                        "CONSTANT_TURN": 0.7,
                                        "HAIRPIN_TURN": 0.3,
                                        "LEFT_RIGHT_TURN": 2
                                }
                        },
                        "Small Straights Slaloms": {
                                "MIN_STRAIGHT": 5,
                                "MAX_STRAIGHT": 40,
                                "MIN_CONSTANT_TURN": 10,
                                "MAX_CONSTANT_TURN": 25,
                                "MIN_HAIRPIN": 4.5,
                                "MAX_HAIRPIN": 10,
                                "MAX_HAIRPIN_PAIRS": 3,
                                "MAX_LENGTH": 700,
                                "LAX_GENERATION": True,
                                "TRACK_WIDTH": 3.5,
                                "COMPONENTS": {
                                        "STRAIGHT": 1,
                                        "CONSTANT_TURN": 0.7,
                                        "HAIRPIN_TURN": 0.3,
                                        "LEFT_RIGHT_TURN": 2,
                                        "SLALOM": 0.3
                                }
                        },
                        "Computer Friendly": {
                                "MIN_STRAIGHT": 10,
                                "MAX_STRAIGHT": 80,
                                "MIN_CONSTANT_TURN": 5,
                                "MAX_CONSTANT_TURN": 15,
                                "MIN_HAIRPIN": 4.5,
                                "MAX_HAIRPIN": 10,
                                "MAX_HAIRPIN_PAIRS": 3,
                                "MAX_LENGTH": 500,
                                "LAX_GENERATION": True,
                                "TRACK_WIDTH": 3.5,
                                "COMPONENTS": {
                                        "STRAIGHT": 1,
                                        "CONSTANT_TURN": 0.7,
                                        "HAIRPIN_TURN": 0.3,
                                        "LEFT_RIGHT_TURN": 2
                                }
                        },
                        "Bezier Small Straights": {
                                "MIN_STRAIGHT": 5,
                                "MAX_STRAIGHT": 40,
                                "MIN_CONSTANT_TURN": 10,
                                "MAX_CONSTANT_TURN": 25,
                                "MIN_HAIRPIN": 4.5,
                                "MAX_HAIRPIN": 10,
                                "MAX_HAIRPIN_PAIRS": 3,
                                "MAX_LENGTH": 700,
                                "LAX_GENERATION": True,
                                "TRACK_WIDTH": 3.5,
                                "COMPONENTS": {
                                        "STRAIGHT": 1,
                                        "CONSTANT_TURN": 0.7,
                                        "HAIRPIN_TURN": 0.3,
                                        "LEFT_RIGHT_TURN": 2,
                                        "BEZIER": 10
                                }
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
                TrackGenerator.MIN_HAIRPIN_PAIRS = (
                        1 if TrackGenerator.MAX_HAIRPIN_PAIRS > 0 else 0
                )
                TrackGenerator.MAX_TRACK_LENGTH = values["MAX_LENGTH"]
                TrackGenerator.LAX_GENERATION = values["LAX_GENERATION"]
                TrackGenerator.component_weighting_dict = collections.defaultdict(
                        lambda: 0,
                        values["COMPONENTS"]
                )
                TrackGenerator.TRACK_WIDTH = values["TRACK_WIDTH"]

        # These three are used as constants for batch generation.
        class GEN_STOP:
            pass

        class GEN_CONTINUE:
            pass

        class GEN_RELEASE:
            pass

        @staticmethod
        def batch(first, subsequent=0, threshold=None, postprocessing=lambda x: x):
                """
                A python coroutine that batches up track generations.

                It generates "first" amount of generations in one batch,
                and create a generator that returns these.

                If "subsequent" is set to something other than 0, then
                whenever the saved-up batches get below "threshold", the
                generator will generate "subsequent" more tracks.

                If "subsequent" is zero, the generation will terminate.

                The coroutine should be sent "GEN_CONTINUE" if it should continue,
                and "GEN_STOP" if it should stop immediately, and "GEN_RELEASE" if
                it should return everything it has queued up so far.

                postprocessing is a function to be applied after normal generation.
                Its intended function is to convert the raw data into track image data.
                """
                GEN_CONTINUE = TrackGenerator.GEN_CONTINUE
                GEN_STOP = TrackGenerator.GEN_STOP
                GEN_RELEASE = TrackGenerator.GEN_RELEASE

                # We do the priming of the coroutine in this function, hence
                # the nested functions here!
                def inner_coroutine(first, subsequent, threshold):
                        # If threshold has no value, set it to check whenever it has
                        # used up "subsequent" generations (so that it is effectively
                        # refilling the batch back up to "first")
                        if threshold is None and subsequent > 0:
                                threshold = first - subsequent + 1
                        elif threshold is None:
                                threshold = -1

                        saved_batch = [
                                postprocessing(TrackGenerator.generate())
                                for x in range(0, first)
                        ]

                        # Add None to the batch so priming it doesn't remove valid data.
                        saved_batch += [None]
                        signal = GEN_CONTINUE

                        while len(saved_batch) > 0:

                                # Handle the coroutine signals passed back in
                                if signal is GEN_STOP:
                                        _ = yield saved_batch
                                        return
                                elif signal is GEN_RELEASE:
                                        signal = yield saved_batch
                                        saved_batch = []
                                elif signal is GEN_CONTINUE:
                                        signal = yield saved_batch.pop()

                                # If needs to refill the batch, refill it!
                                if len(saved_batch) < threshold:
                                        saved_batch = (
                                                [postprocessing(TrackGenerator.generate())
                                                 for x in range(0, subsequent)] + saved_batch
                                        )
                        return
                to_return = inner_coroutine(first, subsequent, threshold)

                # Prime coroutine
                next(to_return)

                return to_return

        @staticmethod
        def generate():
                """
                Generates the track as pure data
                  (a List[Tuple[float,float]] list of points on the track)

                Returns a list of points to define the path of the track,
                  along with a bounding width & height for how big the track is.
                """

                xys = []
                overlapped = False
                failure_count = 0
                while overlapped or xys == []:
                        # Re-generate if the track overlaps itself
                        (xys, twidth, theight) = generate_random_track((0, 0))
                        if twidth == theight and twidth == 0:
                                overlapped = True
                        else:
                                xys2 = compactify_points([
                                        (int(x[0]), int(x[1])) for x
                                        in get_points_from_component_list(xys)
                                ])
                                overlapped = check_if_overlap(xys2)
                        if overlapped:
                                if TrackGenerator.FAILURE_INFO:
                                        rospy.logerr("Overlap check " + str(failure_count) + " failed")
                                failure_count += 1
                        if failure_count > 1000:
                                raise GenerationFailedException

                # Now let's randomly flip it a bit to spice it up
                if random.uniform(0, 1) < 0.5:
                        # flip xys by x
                        xys = [
                                (name, [(-x + twidth, y) for (x, y) in pointlist])
                                for name, pointlist in xys
                        ]
                if random.uniform(0, 1) < 0.5:
                        # flip xys by y
                        xys = [
                                (name, [(x, -y + theight) for (x, y) in pointlist])
                                for name, pointlist in xys
                        ]

                return (xys, twidth, theight)


###################################
# Decorators For Micro Generators #
###################################

def generic_micro(micro_name, allow_double_placement=False):
        """
        Decorator for generic track components

        micro_name: The internal name of the micro.  Should be a string.

        allow_double_placement: Allows this component to be placed back-to-back
        """
        def to_return(decorated_function):
                to_append = (micro_name, decorated_function)
                TrackGenerator.generic_micro_list.append(to_append)
                if allow_double_placement:
                        TrackGenerator.double_placement_set.add(micro_name)
                return decorated_function
        return to_return


def refocus_micro(micro_name, allow_double_placement=False):
        """
        Decorator for refocus track components

        micro_name: The internal name of the micro.  Should be a string.

        allow_double_placement: Allows this component to be placed back-to-back

        Refocus components should have certain tangency
        properties guaranteed.  Consult the wiki for
        an in-depth understanding.
        """
        def to_return(decorated_function):
                to_append = (micro_name, decorated_function)
                TrackGenerator.refocus_micro_list.append(to_append)
                if allow_double_placement:
                        TrackGenerator.double_placement_set.add(micro_name)
                return decorated_function
        return to_return


def connector_micro(micro_name, allow_double_placement=False):
        """
        Decorator for refocus track components

        micro_name: The internal name of the micro.  Should be a string.

        allow_double_placement: Allows this component to be placed back-to-back

        Connector components should have certain tangency
        properties guaranteed.  Consult the wiki for
        an in-depth understanding.
        """
        def to_return(decorated_function):
                to_append = (micro_name, decorated_function)
                TrackGenerator.connector_micro_list.append(to_append)
                if allow_double_placement:
                        TrackGenerator.double_placement_set.add(micro_name)
                return decorated_function
        return to_return


def linear_micro(micro_name,
                 allow_double_placement=False,
                 linearize=False,
                 start=False):
        """
        Decorator for refocus track components

        micro_name: The internal name of the micro.  Should be a string.

        allow_double_placement: Allows this component to be placed back-to-back

        linearize: If True, this adds the "length" parameter automatically
                   by fishing it from the params list of the decorated_function.
                   This allows funtions like linear_straight to be written like
                   a generic_straight, without having to worry about function
                   signature.

        Linear components should have certain tangency
        properties guaranteed.  Consult the wiki for
        an in-depth understanding.
        """
        def to_return(decorated_function):
                if linearize:
                        to_append = (
                                micro_name,
                                get_linearized_function(decorated_function)
                        )
                else:
                        to_append = (micro_name, decorated_function)
                if start:
                        TrackGenerator.start_micro_list.append(to_append)
                TrackGenerator.linear_micro_list.append(to_append)
                if allow_double_placement:
                        TrackGenerator.double_placement_set.add(micro_name)
                return decorated_function

        return to_return


def get_linearized_function(to_linearize):
        """
        Linearizes function according to linearize parameter of linear_micro decorator.
        """
        def linearized_function(point_in, tangent_in, normal_in, length, params={}):
                params["length"] = length
                return to_linearize(point_in, tangent_in, normal_in, params=params)
        return linearized_function


def special_micro(micro_name, cone_placer):
        """
        Decorator for refocus track components

        micro_name:  The internal name of the micro.  Should be a string.

        cone_placer: The function to be used on cone placement.
        """
        def to_return(decorated_function):
                to_append = (micro_name, cone_placer)
                TrackGenerator.special_micro_list.append(to_append)
                return decorated_function
        return to_return


def rare_micro(micro_name, count=1):
        """
        If a micro is made rare, it will only be allowed
        to appear a limited amount of times in a track.

        micro_name: name of the micro that is rare

        count: the amount of times it is allowed to appear
        """
        TrackGenerator.rare_micro_list.append((micro_name, 0, count))

        def to_return(decorated_function):
                return decorated_function
        return to_return


def get_cone_function(micro_name):
        """
        Returns the cone placement function utilized by the micro_name micro generator.
        """
        for name, placer in TrackGenerator.special_micro_list:
                if name == micro_name:
                        return placer

        return cone_default


####################
# Macro Generators #
####################

def is_used_too_much(micro_name):
        """
        Prevents a micro from being used too much
        (can't surpass max_count given in rare_micro_list)
        """
        for name, count, max_count in TrackGenerator.rare_micro_list:
                if name == micro_name and count >= max_count:
                        return True
        return False


def get_random_micro(type_of_micro):
        """
        Gets random micro of micro type "type_of_micro"
        """

        # Get the list of allowed micros
        list_to_check = [
                x for x in TrackGenerator.__dict__[type_of_micro + "_micro_list"]
                if not (
                        x[0] == TrackGenerator.previous_micro and
                        x[0] not in TrackGenerator.double_placement_set
                ) and not (
                        is_used_too_much(x[0])
                )
        ]
        weightings = [
                TrackGenerator.component_weighting_dict[x[0]] for x in list_to_check
        ]

        # In python 3, we would use random.choices(list_to_check, weightings)
        # This doesn't exist in python 2, so for now we have implemented our
        # own version of it.
        # random_choices is defined in LauncherUtilities.py
        return random_choices(list_to_check, weightings)


def use_micro(micro_tuple):
        """
        Takes in a (name,function) tuple like the ones stored in
        TrackGenerator.generic_micro_list

        Changes TrackGenerator.previous_micro to the micro, and returns the function so it
        can be used.
        """

        TrackGenerator.previous_micro = micro_tuple[0]
        return micro_tuple[1]


def register_output(list_to_register_to, points_out):
        """Registers a collection of points according to the micro generated."""
        to_append = (TrackGenerator.previous_micro, points_out)
        list_to_register_to.append(to_append)


def get_generic():
        """Returns a generic micro & logs it as used"""
        return use_micro(get_random_micro("generic"))


def get_refocus():
        """Returns a refocus micro & logs it as used"""
        return use_micro(get_random_micro("refocus"))


def get_connector():
        """Returns a connector micro & logs it as used"""
        return use_micro(get_random_micro("connector"))


def get_linear():
        """Returns a linear micro & logs it as used"""
        return use_micro(get_random_micro("linear"))


def get_start():
        """Returns a linear micro & logs it as used"""
        return use_micro(get_random_micro("start"))


def reset_counts():
        """
        Restarts the counts for each micro
        (We were keeping track of how many times they've been used because
        some have maximum allowed uses - this resets them so that we
        can start a new generation.)
        """
        TrackGenerator.previous_micro = ""
        TrackGenerator.rare_micro_list = [
                (name, 0, max_count) for name, cur_count, max_count
                in TrackGenerator.rare_micro_list
        ]


def get_last_comp_points(comps):
        """Gets last list of point in a list of components"""
        return comps[-1][1]


def generate_random_track(start_point):
        """
        Generates a random track given a starting point.
        """
        reset_counts()
        total_length = 0
        components = []

        # We start with a small linear
        initial_tangent = get_random_unit_vector(0, math.pi/8)
        tangent_in = initial_tangent
        point_in = start_point
        normal_in = get_normal_vector(tangent_in)
        (points_out, tangent_out, normal_out, added_length) = get_start()(
                point_in,
                tangent_in,
                normal_in,
                random.uniform(
                        TrackGenerator.MIN_STRAIGHT,
                        TrackGenerator.MAX_STRAIGHT
                )
        )
        total_length += added_length
        register_output(components, points_out)

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
                prev_components = components

                # Prepare inputs
                tangent_in = tangent_out
                normal_in = normal_out
                point_out = goal_point
                point_in = points_out[-1]

                # Generate from point to point
                (comps, tangent_out, normal_out, added_length) = generate_point_to_point(
                        point_in,
                        point_out,
                        tangent_in,
                        normal_in,
                        20
                )

                # This grabs the last component's points.
                points_out = get_last_comp_points(comps)
                total_length += added_length
                components.extend(comps)

                # Now let's do early-checking for overlaps
                test = compactify_points([
                        (int(x[0]), int(x[1])) for x
                        in get_points_from_component_list(components)
                ])
                if check_if_overlap(test):
                        if TrackGenerator.FAILURE_INFO:
                                rospy.logerr("Early Overlap Checking: Failed")

                        # Generation failed test, undo last bit
                        fails += 1
                        total_length -= added_length
                        components = prev_components

                        if fails == max_fails:
                                return (test, 0, 0)

        # Now lets head back to the start:
        # We're gonna set a checkpoint that is close but not exactly the start point
        # so that we have breathing room for the final manouever:
        directing_point = (start_point[0] - TrackGenerator.MAX_STRAIGHT * 0.5,
                           start_point[1] + TrackGenerator.MAX_CONSTANT_TURN * 2)

        # Prepare inputs
        tangent_in = tangent_out
        normal_in = normal_out
        point_out = directing_point
        point_in = points_out[-1]

        # Generate from point to point
        (comps, tangent_out, normal_out, added_length) = generate_point_to_point(
                point_in,
                point_out,
                tangent_in,
                normal_in,
                0
        )
        total_length += added_length
        components.extend(comps)
        points_out = get_last_comp_points(comps)

        # Now we will add a circle to point directly away from start point
        tangent_in = tangent_out
        normal_in = normal_out
        point_in = points_out[-1]
        tangent_out = initial_tangent

        points_out, tangent_out, normal_out, added_length = get_connector()(
                point_in,
                start_point,
                tangent_in,
                tangent_out,
                normal_in
        )
        register_output(components, points_out)

        # Sometimes the tangents don't actually match up
        # so if that happens, we throw out the track and start anew.
        if get_distance(initial_tangent, tangent_out) > 0.01:
                return generate_random_track(start_point)

        if not TrackGenerator.LAX_GENERATION:
            # Check if whole track is too big
            if total_length > TrackGenerator.MAX_TRACK_LENGTH:
                return generate_random_track(start_point)

        return convert_components_to_all_positive(components)


def generate_point_to_point(point_in,
                            point_out,
                            tangent_in,
                            normal_in,
                            fuzz_radius,
                            depth=20):
        """
        Generates a track from point_in to point_out with incoming tangent tangent_in,
        and incoming normal normal_in.

        Does not necessarily exactly end at point_out, it merely aims to get
        within fuzz_radius.

        depth is the max amout of times it may recurse (this is a recursive function that
        incrementally builds the track up recursion by recursion).

        Outputs (points_out, tangent_out, normal_out, added_length)
        """

        added_length = 0
        components = []

        # We start out by refocusing ourselves towards point_out.
        (points_out, tangent_out, normal_out, delta_length) = (
                get_refocus()(
                        point_in,
                        point_out,
                        tangent_in,
                        normal_in,
                )
        )
        added_length += delta_length
        register_output(components, points_out)

        # Prepare data for next component
        point_in = points_out[-1]
        tangent_in = tangent_out
        normal_in = normal_out

        # Now we want to know how close we are to the goal - if we're close enough
        # to just directly draw a straight path to it.
        distance_from_end = get_distance(point_in, point_out)
        if distance_from_end <= TrackGenerator.MAX_STRAIGHT + fuzz_radius:
                straight_length = min(distance_from_end, TrackGenerator.MAX_STRAIGHT)
                (points_out, tangent_out, normal_out, delta_length) = get_linear()(
                        point_in,
                        tangent_in,
                        normal_in,
                        straight_length
                )
                added_length += delta_length
                register_output(components, points_out)

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
                (points_out, tangent_out, normal_out, delta_length) = get_linear()(
                        point_in,
                        tangent_in,
                        normal_in,
                        straight_length
                )
                added_length += delta_length
                register_output(components, points_out)

                # Prepare data for next component
                point_in = points_out[-1]
                tangent_in = tangent_out
                normal_in = normal_out

                (points_out, tangent_out, normal_out, delta_length) = (
                        get_generic()(
                                point_in,
                                tangent_in,
                                normal_in
                        )
                )
                added_length += delta_length
                register_output(components, points_out)

                # Prepare data for next component
                point_in = points_out[-1]
                tangent_in = tangent_out
                normal_in = normal_out

                # Now, we recurse
                if depth > 0:
                        (comps, tangent_out, normal_out, delta_length) = (
                                generate_point_to_point(
                                        point_in,
                                        point_out,
                                        tangent_in,
                                        normal_in,
                                        fuzz_radius,
                                        depth=depth - 1,
                                )
                        )
                        added_length += delta_length
                        components.extend(comps)

        return (components, tangent_out, normal_out, added_length)


####################
# Micro Generators #
####################
#
# There are six types of micro generators (generators can have multiple types):
#
# 1: "generic"
#         These have the signature generic_<type>(point_in,tangent_in,normal_in,params)
#         and output (parametrization_out, tangent_out, normal_out, added_length)
#
#         Generics just create a generically random component, with no extra requirements.
#
# 2: "refocus"
#         These have the signature
#                 refocus_<type>(point_in,point_out,tangent_in,normal_in,params)
#         with the same output as generics.
#
#         Refocusers ensure that the outgoing tangent points towards point_out
#
# 3: "connector"
#         These have the signature:
#                 connector_<type>(point_in,point_out,tangent_in,tangent_out,normal_in,params)
#         with the same output as generics
#
#         Connectors ensure that the component ends at point_out,
#         with outgoing tangent tangent_out
#
# 4: "linear"
#         These have the signature:
#                 linear_<type>(point_in,tangent_in,normal_in,length,params)
#         with the same output as generics
#
#         If we consider the component to end at "point_out", then linear components
#         guarantee that point_out will lie on the ray emanating from point_in in the
#         direction of tangent_in.  The outgoing tangent will be guaranteed to be
#         the same as tangent_in.
#
#         The length parameter of the signature can be ommited if the "linearize" option
#         of the linear_micro decorator is ommitted, provided that one of the params{}
#         of the component is named "length".
#
# 5: "special"
#         There is no set signature for special micros.  They are identified solely
#         by the presence of the special_micro decorator.  Special micros delegate to
#         a special cone-placement function rather than the normal one.
#
# 6: "rare"
#         Similar to special, rare tells the generator that this specific component
#         is only allowed to appear a couple times.
#
# extra: "parametric"
#         These have variable signatures, but always output a function of one variable
#         which ranges from 0 to 1, parametrizing the curve they represent in terms of it.
#
#         They aren't really micro generators, but are crucial helper functions for them.
#
# extra: "cone"
#         These are used to define cone placements, like cone_default and cone_slalom
#         They always take in a list of points, and return a list of triples (px, py, color)
#         of the (px, py) position of a cone of color color that will be placed by the
#         caller of the function.
#
#########################################################

def de_parameterize(func):
        """Given a parametric function, turn it into a list of points"""
        return [func(1.0 * t / (TrackGenerator.FIDELITY - 1))
                for t in range(0, TrackGenerator.FIDELITY)]


# Flags for cone placement micros
class CONE_INNER:
    pass


class CONE_OUTER:
    pass


class CONE_ORANGE:
    pass


class CONE_START:
    pass


class CONE_BIG_ORANGE:
    pass


class NOISE:
    pass


def cone_start(xys, track_width=None):
    """Wrapper for cone_default with start parameter"""
    return cone_default(
            xys,
            starting=True,
            track_width=track_width,
            slalom=False,
            prev_points=None
    )


def cone_slalom(xys, track_width=None, prev_points=None):
        """Wrapper for cone_default with slalom parameter"""
        return cone_default(
                xys,
                starting=False,
                track_width=track_width,
                slalom=True,
                prev_points=prev_points
        )


def cone_default(xys, starting=False, track_width=None, slalom=False, prev_points=None):
        """
        Default cone placement algorithm, can be overriden on a micro by using the
        special micro decorator.

        Takes in a list of points, returns a list of triples dictating x, y position
        of cone of color color [(x, y, color)].

        Optionally takes a flag "starting" to make the first pair of cones orange,
        and optional "track_width" to override TrackGenerator.TRACK_WIDTH, which
        is useful if accessing ConversionTools.xys_to_png from a non-generator context.
        """

        if track_width is None:
                cone_normal_distance_parameter = TrackGenerator.TRACK_WIDTH
        else:
                cone_normal_distance_parameter = track_width

        # Hardcoded parameters (not available in launcher because the set of
        # well-behaved answers is far from dense in the problemspace):

        # How close can cones be from those on opposite side.
        cone_cross_closeness_parameter = cone_normal_distance_parameter * 3 / 4 - 1

        # Larger numbers makes us check more parts of the track for conflicts.
        cone_check_amount = 30

        # How close can cones be from those on the same side.
        cone_adjacent_closeness_parameter = 4

        all_points_north = []
        all_points_south = []
        prev_points_north = [(-10000, -10000)]
        prev_points_south = [(-10000, -10000)]
        to_return = []

        # Process the previous points down to a manageable amount.
        if prev_points is not None:
                first_orange_cones = [
                        (x, y) for x, y, c in prev_points
                        if c is CONE_ORANGE
                ][:2]
                all_points_south = [
                        (x, y) for x, y, c in prev_points
                        if c is CONE_INNER
                ]
                all_points_south = (
                        all_points_south[-cone_check_amount:] + first_orange_cones
                )
                all_points_north = [
                        (x, y) for x, y, c in prev_points
                        if c is CONE_OUTER
                ]
                all_points_north = (
                        all_points_north[-cone_check_amount:] + first_orange_cones
                )

        # This is used to check if the yellow and blue cones suddenly swapped.
        last_tangent_normal = (0, 0)

        # Used to make sure orange cones start ahead of the car
        orig_tangent = (0, 0)

        for i in range(len(xys)):
                # Skip first part [as hard to calculate tangent]
                if i == 0:
                    continue

                # The idea here is to place cones along normals to the
                # tangent at any given point,
                # while making sure they aren't too close.

                # Here we calculate the normal vectors along the track
                # As we want cones to be placed along the normal.
                cur_point = xys[i]
                cur_tangent_angle = calculate_tangent_angle(xys[:(i+1)])
                if i == 1:
                    orig_tangent = (math.cos(cur_tangent_angle), math.sin(cur_tangent_angle))
                cur_tangent_normal = (
                    math.ceil(
                        cone_normal_distance_parameter * math.sin(cur_tangent_angle)
                    ),
                    math.ceil(
                        -cone_normal_distance_parameter * math.cos(cur_tangent_angle)
                    )
                )

                # Check if normal direction suddenly flipped relative to last iteration.
                # (And reverse it if it happened)
                dot_product = (
                        last_tangent_normal[0] * cur_tangent_normal[0] +
                        last_tangent_normal[1] * cur_tangent_normal[1]
                )

                # Tangent flipped!
                # (As when dot product is negative, vectors are opposite)
                if dot_product < 0:
                        cur_tangent_normal = scale_vector(cur_tangent_normal, -1)

                last_tangent_normal = cur_tangent_normal

                # This is where the cones will be placed, provided they pass the
                # distance checks later.
                north_point = ((cur_point[0] + cur_tangent_normal[0]),
                               (cur_point[1] + cur_tangent_normal[1]))
                south_point = ((cur_point[0] - cur_tangent_normal[0]),
                               (cur_point[1] - cur_tangent_normal[1]))

                # Calculates shortest distance to cone on same side of track
                difference_from_prev_north = min([
                                                  (north_point[0] - prev_point_north[0])**2 +
                                                  (north_point[1] - prev_point_north[1])**2
                                                  for prev_point_north in prev_points_north
                ])
                difference_from_prev_south = min([
                                                  (south_point[0] - prev_point_south[0])**2 +
                                                  (south_point[1] - prev_point_south[1])**2
                                                  for prev_point_south in prev_points_south
                ])

                # Calculates shortest distance to cone on different side of track
                cross_distance_ns = min([
                                         (north_point[0] - prev_point_south[0])**2 +
                                         (north_point[1] - prev_point_south[1])**2
                                         for prev_point_south in prev_points_south
                ])
                cross_distance_sn = min([
                                         (south_point[0] - prev_point_north[0])**2 +
                                         (south_point[1] - prev_point_north[1])**2
                                         for prev_point_north in prev_points_north
                ])

                # Here we ensure cones don't get too close to the track
                rel_xys = xys[
                    max([0, i - cone_check_amount]):
                    min([len(xys), i + cone_check_amount])
                ]
                distance_pn = min([
                                   (north_point[0] - xy[0])**2 +
                                   (north_point[1] - xy[1])**2
                                   for xy in rel_xys
                ])
                distance_ps = min([
                                   (south_point[0] - xy[0])**2 +
                                   (south_point[1] - xy[1])**2
                                   for xy in rel_xys
                ])

                # And we consider all these factors to see if the cones are viable
                north_viable = (
                      difference_from_prev_north > cone_adjacent_closeness_parameter**2 and
                      cross_distance_ns > cone_cross_closeness_parameter**2 and
                      distance_pn > cone_cross_closeness_parameter**2
                )
                south_viable = (
                                difference_from_prev_south > cone_adjacent_closeness_parameter**2 and
                                cross_distance_sn > cone_cross_closeness_parameter**2 and
                                distance_ps > cone_cross_closeness_parameter**2
                )

                # And when they are viable, draw them!
                if (north_viable):
                        px_x = (north_point[0])
                        px_y = (north_point[1])
                        to_return.append((px_x, px_y, CONE_OUTER))
                        all_points_north.append(north_point)
                if (south_viable):
                        px_x = (south_point[0])
                        px_y = (south_point[1])
                        to_return.append((px_x, px_y, CONE_INNER))
                        all_points_south.append(south_point)

                # Handle placement of slalom cones
                if slalom and (north_viable or south_viable):
                        to_return.append(((cur_point[0]), (cur_point[1]), CONE_ORANGE))

                # Only keep track of last couple of previous cones
                # (and the very first one, for when the loop joins up)
                # Specifically, if we assume cone_check_amount is 30, then
                # we have prev_points north keep track of the most recent 30 cones,
                # and also the very first start cone.
                # This is because we want to make sure no cones are too close to eachother,
                # but checking all cones would be too expensive.
                # The first cone is kept track of because the track is a loop so
                # it needs to look ahead a bit.
                if len(all_points_north) > cone_check_amount:
                        prev_points_north = (all_points_north[-cone_check_amount:] + [all_points_north[0]])
                elif len(all_points_north) > 0:
                        prev_points_north = all_points_north

                if len(all_points_south) > cone_check_amount:
                        prev_points_south = (all_points_south[-cone_check_amount:] + [all_points_south[0]])
                elif len(all_points_south) > 0:
                        prev_points_south = all_points_south

        if starting:
                # Add orange start cones
                for point_list in [all_points_north, all_points_south]:
                        (i, j) = point_list[0]
                        for idx, tup in enumerate(to_return):
                                x, y, _ = tup
                                if int(x) == int(i) and int(y) == int(j):
                                        to_return[idx] = (
                                                (x + point_list[1][0])/2,
                                                (y + point_list[1][1])/2,
                                                CONE_START
                                        )

        return to_return


@special_micro("SLALOM", cone_slalom)
@linear_micro("SLALOM", linearize=True)
@generic_micro("SLALOM")
@linear_micro("STRAIGHT", linearize=True, start=True)
@generic_micro("STRAIGHT")
def generic_straight(point_in,
                     tangent_in,
                     normal_in,
                     params={}):
        """
        Creates a straight line micro generator

        params["length"]: specifies how long to make the straight
        """

        # Load in params
        if "length" in params:
                length = params["length"]
        else:
                length = random.uniform(
                        TrackGenerator.MIN_STRAIGHT,
                        TrackGenerator.MAX_STRAIGHT
                )

        # Prepare outputs
        straight_func = parametric_straight(tangent_in, point_in, length)
        tangent_out = tangent_in
        normal_out = normal_in
        added_length = length

        return (
                de_parameterize(straight_func),
                tangent_out,
                normal_out,
                added_length
        )


@generic_micro("CONSTANT_TURN", allow_double_placement=True)
def generic_constant_turn(point_in,
                          tangent_in,
                          normal_in,
                          params={}):
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
                turn_against_normal = random.uniform(0, 1) < 0.5

        if turn_against_normal:
                # We need to flip our reference frame.
                normal_in = scale_vector(normal_in, -1)

        if "radius" in params:
                radius = params["radius"]
        else:
                radius = random.uniform(
                        TrackGenerator.MIN_CONSTANT_TURN,
                        TrackGenerator.MAX_CONSTANT_TURN
                )

        if "circle_percent" in params:
                circle_percent = params["circle_percent"]
        else:
                # Note: if circles turn too much,
                # risk of self-intersection rises dramatically.
                circle_percent = random.uniform(0.1, 0.5)

        center = add_vectors(point_in, scale_vector(normal_in, radius))

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
        normal_out = normalize_vec((
                points_out[-1][0] - center[0],
                points_out[-1][1] - center[1]
        ))

        # And finally recalculate the tangent:
        tangent_out = calculate_tangent_vector(points_out)

        # Returns a list of points and the new edge of the racetrack and the change in length
        return (points_out, tangent_out, normal_out, added_length)


@rare_micro("HAIRPIN_TURN", count=2)
@generic_micro("HAIRPIN_TURN")
def generic_hairpin_turn(point_in,
                         tangent_in,
                         normal_in,
                         params={}):
        """
        Creates a hairpin micro generator.

        params["turn_against_normal"]: boolean, when true then the
                                       hairpin will have the opposite normal.

        params["radius"]: radius of the circle

        params["uniform_circles"]: if True, all circles have the
                                            same radius (params["radius"])
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
                turn_against_normal = random.uniform(0, 1) < 0.5

        if turn_against_normal:
                # We need to flip our reference frame.
                normal_in = scale_vector(normal_in, -1)

        if "radius" in params:
                radius = params["radius"]
        else:
                radius = random.uniform(
                        TrackGenerator.MIN_CONSTANT_TURN,
                        TrackGenerator.MAX_CONSTANT_TURN
                )

        if "uniform_circles" in params:
                uniform_circles = params["uniform_circles"]
        else:
                uniform_circles = random.uniform(0, 1) < 0.5

        if "straight_length" in params:
                straight_length = params["straight_length"]
        else:
                straight_length = random.uniform(
                        TrackGenerator.MIN_STRAIGHT,
                        TrackGenerator.MAX_STRAIGHT
                )

        if "wobbliness" in params:
                wobbliness = params["wobbliness"]
        else:
                wobbliness = random.uniform(0.45, 0.55)

        if "switchbacks" in params:
                switchbacks = params["switchbacks"]
        else:
                switchbacks = 2 * random.randrange(
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
                        radius = random.uniform(
                                TrackGenerator.MIN_CONSTANT_TURN,
                                TrackGenerator.MAX_CONSTANT_TURN
                        )

                # Do the circle part
                (points_out, tangent_out, normal_out, delta_length) = generic_constant_turn(
                        point_in,
                        tangent_in,
                        normal_in,
                        params={
                                "turn_against_normal": True,
                                "circle_percent":      wobbliness,
                                "radius":              radius
                        }
                )
                added_length += delta_length
                xys.extend(points_out)

                # Prepare for next component
                point_in = points_out[-1]
                tangent_in = tangent_out
                normal_in = normal_out

                # Now tack on the straight
                (points_out, tangent_out, normal_out, delta_length) = generic_straight(
                        point_in,
                        tangent_in,
                        normal_in,
                        params={
                                "length": straight_length
                        }
                )
                added_length += delta_length
                xys.extend(points_out)

                # Prepare for next component
                # For hairpins, normals should always flip!
                # That way they zigzag
                point_in = points_out[-1]
                tangent_in = tangent_out
                normal_in = scale_vector(normal_out, -1)

        return (xys, tangent_out, normal_out, added_length)


@refocus_micro("CONSTANT_TURN")
def refocus_constant_turn(point_in,
                          point_out,
                          tangent_in,
                          normal_in,
                          params={}):
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
                turn_against_normal = random.uniform(0, 1) < 0.5

        if turn_against_normal:
                # We need to flip our reference frame.
                normal_in = scale_vector(normal_in, -1)

        if "radius" in params:
                radius = params["radius"]
        else:
                radius = random.uniform(
                        TrackGenerator.MIN_CONSTANT_TURN, TrackGenerator.MAX_CONSTANT_TURN
                )

        if "recursed" in params:
                recursed = params["recursed"]
        else:
                recursed = False

        center = add_vectors(point_in, scale_vector(normal_in, radius))

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
        def intermediate_point(s, c, a):
                (sx, sy) = s
                (cx, cy) = c
                cos_a = math.cos(a)
                sin_a = math.sin(a)
                del_x = sx - cx
                del_y = sy - cy
                result_x = cos_a * del_x - sin_a * del_y + cx
                result_y = sin_a * del_x + cos_a * del_y + cy
                return (result_x, result_y)
        circle_func = Parametrization(lambda a: intermediate_point(point_in, center, a))

        # We are going to iteratively walk around the circle until we are facing the right way
        points_out = []
        max_range = TrackGenerator.FIDELITY
        step_size = 1.0 / max_range
        angle = 0
        for t in range(0, max_range):
                angle = t * step_size * 2 * math.pi
                points_out.append(
                        circle_func(angle*handedness)
                )
                if t != 0:
                        # Check if we're pointing in the right direction
                        # This equates to checking if end_point lies on the
                        # line at points[-1] with the appropriate tangent.
                        (sx, sy) = points_out[-1]
                        (ex, ey) = point_out
                        appropriate_angle = calculate_tangent_angle(points_out)
                        if t > 0.8 * max_range and not recursed:
                                # Very big turn, we don't like that!
                                # We'll just turn the other way instead
                                return refocus_constant_turn(
                                        point_in,
                                        point_out,
                                        tangent_in,
                                        normal_in,
                                        params={
                                                "recursed": True,
                                                "radius": radius,
                                                "turn_against_normal": True
                                        }
                                )
                        if abs(
                                cap_angle(appropriate_angle) - cap_angle(math.atan2((ey - sy), (ex - sx)))
                        ) < 0.01:
                                # Would do equality checking but limited precision,
                                # we just check if close!
                                break

        # Length of circle is, fortunately, easy!  It's simply radius*angle
        added_length = angle*radius

        # Now we want to find the new normal vector,
        normal_out = normalize_vec((
                points_out[-1][0] - center[0],
                points_out[-1][1] - center[1]
        ))

        # And finally recalculate the tangent:
        tangent_out = calculate_tangent_vector(points_out)

        # Returns a list of points and the new edge of the racetrack and the change in length
        return (points_out, tangent_out, normal_out, added_length)


@connector_micro("CONSTANT_TURN")
def connector_constant_turn(point_in,
                            point_out,
                            tangent_in,
                            tangent_out,
                            normal_in,
                            params={}):
        """
        Complicated connector function that attempts to abide by contest regulations.

        Warning: Sometimes the circles are not of allowed radius!
        """

        xys = []
        total_length = 0
        final_tangent_out = tangent_out
        tangent_out = scale_vector(tangent_out, -1)

        # We need to calculate the turn angle (angle between 2 vectors):
        outer_turn_angle = math.acos(
                - tangent_in[0] * tangent_out[0] - tangent_in[1] * tangent_out[1]
        )
        circle_turn_angle = math.pi - outer_turn_angle
        circle_turn_percent = circle_turn_angle / (2 * math.pi)
        circle_radius = random.uniform(
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
        tangent_in = tangent_out
        normal_in = normal_out
        point_in = points_out[-1]
        tangent_out = final_tangent_out
        diff = subtract_vectors(point_in, point_out)
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
        normal_in = normal_out
        point_in = points_out[-1]
        straight_length = get_distance(point_in, point_out) * 1.1
        (points_out, tangent_out, normal_out, added_length) = generic_straight(
                point_in,
                tangent_in,
                normal_in,
                params={
                        "length": straight_length
                }
        )
        total_length += added_length
        xys.extend(points_out)

        return (xys, tangent_out, normal_out, total_length)


@generic_micro("LEFT_RIGHT_TURN")
def generic_left_right_turn(point_in,
                            tangent_in,
                            normal_in,
                            params={}):
        """
        Creates a hairpin micro generator.

        This is just the composition of two turns to create a winding road.
        """

        added_length = 0
        xys = []

        # First turn
        (points_out, tangent_out, normal_out, delta_length) = (
                generic_constant_turn(
                        point_in,
                        tangent_in,
                        normal_in,
                        params={
                                "circle_percent": random.uniform(0.1, 0.3)
                        }
                )
        )
        added_length += delta_length
        xys.extend(points_out)

        # Prepare data for next component
        point_in = points_out[-1]
        tangent_in = tangent_out
        normal_in = normal_out

        # And now let's add another circle turning the other way
        # so that we make the track more "winding".
        (points_out, tangent_out, normal_out, delta_length) = (
                generic_constant_turn(
                        point_in,
                        tangent_in,
                        normal_in,
                        params={
                                "circle_percent": random.uniform(0.1, 0.3)
                        }
                )
        )
        added_length += delta_length
        xys.extend(points_out)

        return (xys, tangent_out, normal_out, added_length)


@connector_micro("BEZIER")
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
                scale_in = random.uniform(10, 100)
        if "scale_out" in params:
                scale_out = params["scale_out"]
        else:
                scale_out = random.uniform(10, 100)

        # The incoming tangent is the same as the line from P0 to P1
        # Outgoing tangent is the same as the line from P(n-1) to P(n)
        # Where P0, P1, ..., P(n-1), P(n) are the control points
        # All other control points are free to be selected.
        p0_to_p1 = scale_vector(tangent_in, scale_in)
        p0 = point_in
        p1 = (p0[0] + p0_to_p1[0], p0[1] + p0_to_p1[1])

        pn_1_to_pn = scale_vector(tangent_out, scale_out)
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
        def to_return(cp, t):
                the_sum_x = 0
                the_sum_y = 0
                n = len(cp)
                for i in range(n):
                        coefficient = binom(n-1, i) * (1 - t)**(n - i - 1) * t**i
                        the_sum_x += coefficient * cp[i][0]
                        the_sum_y += coefficient * cp[i][1]
                return (the_sum_x, the_sum_y)
        return Parametrization(lambda t: to_return(control_points, t))


def parametric_straight(slope_vec, start_point, line_length):
        """Returns the parametric function of a line given a slope and start point"""
        def to_return(slope, start, length, t):
                return add_vectors(start, scale_vector(slope, 1.0 * t * line_length))
        return Parametrization(lambda t: to_return(slope_vec, start_point, line_length, t))


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
                cos_a = math.cos(a)
                sin_a = math.sin(a)
                del_x = sx - cx
                del_y = sy - cy
                result_x = cos_a * del_x - sin_a * del_y + cx
                result_y = sin_a * del_x + cos_a * del_y + cy
                return (result_x, result_y)
        return Parametrization(lambda t: output(start_point, center_point, t * delta_angle))
