import math
from random import uniform
import rospy


def calculate_tangent_angle(xys):
        """
        Calculate direction of outgoing tangent of a set of points
        Is an angle!
        """
        sx = xys[-2][0]
        sy = xys[-2][1]
        ex = xys[-1][0]
        ey = xys[-1][1]
        return math.atan2((ey - sy), (ex - sx))


def calculate_tangent_vector(xys):
        angle = calculate_tangent_angle(xys)
        return (math.cos(angle), math.sin(angle))


def cap_angle(ang):
        """Returns angle between 0 and 2*math.pi"""
        if ang < 0:
                return cap_angle(ang + 2 * math.pi)
        elif ang >= 2*math.pi:
                return cap_angle(ang - 2 * math.pi)
        return ang


def cap_angle_odd(ang):
        """Returns angle between -math.pi and math.pi"""
        ang = cap_angle(ang)
        if ang > math.pi:
                ang = ang - 2 * math.pi
        return ang


def magnitude(vec):
        """Calculates magnitude of (x,y) tuple as if it were a vector"""
        (a, b) = vec
        return math.sqrt(a * a + b * b)


def normalize_vec(vec):
        """
        Calculates unit vector pointing in the direction
        that the input tuple would be if it were a vector
        """
        (a, b) = vec
        mag = math.sqrt(a * a + b * b)
        return (a / mag, b / mag)


def get_random_unit_vector(lower_bound=0, upper_bound=2 * math.pi):
        """Returns a unit vector facing in a random direction"""
        rand_angle = uniform(lower_bound, upper_bound)
        return (math.cos(rand_angle), math.sin(rand_angle))


def add_vectors(a, b):
        """Given two tuples a and b, add them together"""
        return (a[0] + b[0], a[1] + b[1])


def scale_vector(vec, scale):
        """Multiplies vec by scale"""
        return (vec[0] * scale, vec[1] * scale)


def get_normal_vector(vec):
        """Given a vector, return a vector normal to it"""
        return (-vec[1], vec[0])


def subtract_vectors(a, b):
        """Given two tuples a and b, find a-b"""
        return (a[0] - b[0], a[1] - b[1])


def get_distance(a, b):
        """Returns the distance between two points"""
        return magnitude(subtract_vectors(a, b))


def convert_points_to_all_positive(xys):
        """
        If the track dips to the negative side of the x or y axes, shift everything over
        Returns shifted points tupled with the range over which the points span
        We also want everything converted to an integer!
        """
        max_neg_x = 0
        max_neg_y = 0
        max_x = 0
        max_y = 0

        for point in xys:
                (x, y) = point
                max_neg_x = min(x, max_neg_x)
                max_neg_y = min(y, max_neg_y)
                max_x = max(x, max_x)
                max_y = max(y, max_y)

        new_xys = []
        padding = 10
        for point in xys:
                (x, y) = point
                new_xys.append(((x - max_neg_x) + padding, (y - max_neg_y) + padding))

        return (new_xys, int(max_x - max_neg_x) + 2 * padding, int(max_y - max_neg_y) + 2 * padding)


def convert_components_to_all_positive(components):
        """
        Like convert_points_to_all_positive, but given a list of components rather than points

        If the track dips to the negative side of the x or y axes, shift everything over
        Returns shifted points tupled with the range over which the points span
        We also want everything converted to an integer!
        """

        _xys = [comp[1] for comp in components]
        xys = []
        for _xy in _xys:
                for xy in _xy:
                        xys.append(xy)

        max_neg_x = 0
        max_neg_y = 0
        max_x = 0
        max_y = 0

        for point in xys:
                (x, y) = point
                max_neg_x = min(x, max_neg_x)
                max_neg_y = min(y, max_neg_y)
                max_x = max(x, max_x)
                max_y = max(y, max_y)

        new_xys = []
        padding = 10
        for comp in components:
                plist = []
                for point in comp[1]:
                        (x, y) = point
                        plist.append(((x - max_neg_x) + padding, (y - max_neg_y) + padding))
                new_xys.append((comp[0], plist))
        return (new_xys, int(max_x - max_neg_x) + 2 * padding, int(max_y - max_neg_y) + 2 * padding)


def compactify_points(points):
        """
        Given a list of points, if any two adjacent points are the
        same after conversion to int, then remove one of them
        """
        remove_list = []
        prev_point = (-10000, -10000)

        def make_int(tup):
                return (int(tup[0]), int(tup[1]))

        for a in range(0, len(points)):
                if (make_int(points[a]) == make_int(prev_point)):
                        remove_list.append(a)
                prev_point = points[a]
        for index in sorted(remove_list, reverse=True):
                del points[index]
        return points


def check_if_overlap(points):
        """
        Naive check to see if track overlaps itself

        (Won't catch overlaps due to track width, only if track center overlaps)
        """

        # Remove end points as in theory that should also be the start point
        # (I remove extra to be a little generous to it as a courtesy)
        points = points[:-10]
        # We want to add in the diagonally-connected points, otherwise you can imagine
        # that two tracks moving diagonally opposite could cross eachother inbetween the pixels,
        # fooling our test.

        for index in range(1, len(points)):
                (sx, sy) = points[index - 1]
                (ex, ey) = points[index]
                manhattan_distance = abs(ex - sx) + abs(ey - sy)
                if (manhattan_distance > 1):
                        # moved diagonally, insert an extra point for it at the end!
                        points.append((sx + 1, sy) if ex > sx else (sx - 1, sy))
        return len(set(points)) != len(points)


def random_choices(list_to_check, weightings):
        """
        Chooses a random element from list_to_check based on weightings.

        Same functionality as random.choices, which does not exist in python 2.
        """

        # We will choose a random number from 0 to the sum of the weightings,
        # and then loop through the weightings and check partial sums of weightings.
        # If the partial sum is strictly more than random_value, then we stop and return
        # the element that pushed the sum over the edge.
        # Strict inequalities are necessary to avoid 0-weighted things getting chosen.
        random_value = uniform(0, sum(weightings))

        partial_sum = 0
        for index, weight in enumerate(weightings):
                partial_sum += weight
                if random_value < partial_sum:
                        return list_to_check[index]

        # As a default, return the last element in the list
        # This is due to uniform() returning a number inclusive to the bounds, along
        # with strict inequality testing. (which means it is possible random_value
        # equals sum(weightings) and thus won't be considered due to strict inequalities.
        return [x for idx, x in enumerate(list_to_check) if weightings[idx] > 0][-1]


def get_points_from_component_list(comp_list):
        """Grabs a list of points from a list of components"""
        flat_list = []
        for sublist in (points for name, points in comp_list):
                for item in sublist:
                        flat_list.append(item)
        return flat_list
