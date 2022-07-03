import random
import math
import cmath
import numpy as np
from os.path import exists


class TrackGenerator:
    def __init__(self, config):
        default_cfg = {
            'seed': random.random(),
            'min_corner_radius': 3,
            'max_frequency': 7,
            'amplitude': 1 / 3,
            'check_self_intersection': True,
            'starting_amplitude': 0.4,
            'rel_accuracy': 0.005,
            'margin': 0,
            'starting_straight_length': 6,
            'starting_straight_downsample': 2,
            'min_cone_spacing': 3 * math.pi / 16,
            'max_cone_spacing': 5,
            'track_width': 3,
            'cone_spacing_bias': 0.5,
            'starting_cone_spacing': 0.5
        }
        self.config = {**default_cfg, **config}

        if 'resolution' not in self.config:
            if 'length' in self.config:
                length = self.config['length']
            else:
                # This formula for approximating the maximum length was derived experimentally
                t = self.config['amplitude'] * self.config['max_frequency']
                length = ((0.6387 * t + 43.86) * t + 123.1) * t + 35.9

            min_sep = self.config['min_cone_spacing']
            max_sep = self.config['max_cone_spacing']
            r = math.log2(length) / self.config['min_corner_radius']
            self.config['resolution'] = int(4 * length * max(1 / min_sep, r / max_sep))

        self.rng = random.Random(self.config['seed'])

    # Path Generation

    @staticmethod
    def _compute_corner_radii(dt, dPdt):
        ddPdt = np.append(np.diff(dPdt), dPdt[0] - dPdt[-1]) / dt
        return abs(dPdt)**3 / (np.conj(dPdt) * ddPdt).imag

    @staticmethod
    def generate_path_w_params(
        rng,
        n_points,
        min_corner_radius,
        max_frequency,
        amplitude=1 / 3
    ):
        """
        Generates a random racetrack. Consider using `generate_path_w_length()`
        instead as it has an easier to use parameter set

        Returns:
        A tuple containing points, normals, and corner radii along the path

        Arguments:
        rng                 -- random number generator used to generate phases
        n_points            -- the sampling resolution. (i.e. the size of the
                                returned position, normal, and corner radii lists)
        min_corner_radius   -- the minimum corner radius. Since the track is scaled
                                to achieve the minimum radius, increasing this will
                                also increase the track length

        max_frequency       -- The highest integer frequency component in the path.
                                Increasing this increases the track length.
        amplitude           -- The amplitude of the waves the make up the path.
                                Increasing this increases the track length.

        See the documentation for more details on the max_frequency and amplitude
        parameters
        """
        # sample around the unit circle
        z = np.array([cmath.exp(2j * math.pi * t / n_points) for t in range(n_points)])

        waves = np.zeros(n_points, dtype=np.complex)
        dwaves = np.zeros(n_points, dtype=np.complex)

        for frequency in range(2, max_frequency + 1):
            # add new term
            phase = cmath.exp(2j * math.pi * rng.random())
            z_pow = z**frequency
            waves += z * (z_pow / (phase * (frequency + 1)) + phase / (z_pow * (frequency - 1)))
            dwaves += z_pow / phase - phase / z_pow

        # generate points
        points = z + amplitude * waves
        dPdt = (1j * z) * (1 + amplitude * dwaves)
        normals = 1j * dPdt / abs(dPdt)
        corner_radii = TrackGenerator._compute_corner_radii(2 * math.pi / n_points, dPdt)

        # scale path so the sharpest corner has a corner radius of min_corner_radius
        scale = min_corner_radius / min(abs(corner_radii))

        return scale * points, normals, scale * corner_radii

    @staticmethod
    def generate_path_w_length(
        rng,
        n_points,
        min_corner_radius,
        margin,
        target_track_length,
        rel_accuracy=0.005,
        starting_amplitude=0.4
    ):
        """
        Generates a random racetrack

        Returns:
        A tuple containing points, normals, and corner radii along the path

        Arguments:
        rng                 -- random number generator used to generate phases
        n_points            -- the sampling resolution. (i.e. the size of the
                                returned position, slope, and corner radii lists)
        min_corner_radius   -- the minimum corner radius
        margin              -- minimum margin on either side of the track
        target_track_length -- the track length
        rel_accuracy        -- the maximum relative error in the track length
        starting_amplitude  -- the initial amplitude estimate and also the maximum amplitude
        """
        # sample around the unit circle
        z = np.array([cmath.exp(2j * math.pi * t / n_points) for t in range(n_points)])

        waves = np.zeros(n_points, dtype=np.complex)
        dwaves = np.zeros(n_points, dtype=np.complex)

        frequency = 1
        amplitude = starting_amplitude

        while True:
            # add more terms until the track length is greater than the target length
            while True:
                # add new term
                frequency += 1
                phase = cmath.exp(2j * math.pi * rng.random())
                z_pow = z**frequency
                waves += z * (z_pow / (phase * (frequency + 1)) + phase / (z_pow * (frequency - 1)))
                dwaves += z_pow / phase - phase / z_pow

                # generate points
                points = z + amplitude * waves
                dPdt = (1j * z) * (1 + amplitude * dwaves)
                corner_radii = TrackGenerator._compute_corner_radii(2 * math.pi / n_points, dPdt)

                # scale path to achieve a minimum corner radius of min_corner_radius
                scale = min_corner_radius / min(abs(corner_radii))
                track_length = scale * sum(abs(np.diff(points)))

                if track_length >= target_track_length:
                    break

            # find amplitude that results in a track_length of target_track_length
            upper_amp = amplitude
            lower_amp = 0
            upper_offset = track_length - target_track_length
            lower_offset = 2 * math.pi * min_corner_radius - target_track_length

            while abs(track_length - target_track_length) / target_track_length > rel_accuracy:
                # linearly interpolate between the lower and upper bounds
                amplitude = ((lower_offset * upper_amp - upper_offset * lower_amp)
                             / (lower_offset - upper_offset))

                # generate points
                points = z + amplitude * waves
                dPdt = (1j * z) * (1 + amplitude * dwaves)
                corner_radii = TrackGenerator._compute_corner_radii(2 * math.pi / n_points, dPdt)

                # scale path to achieve a minimum corner radius of min_corner_radius
                scale = min_corner_radius / min(abs(corner_radii))
                track_length = scale * sum(abs(np.diff(points)))

                if track_length < target_track_length:
                    lower_amp = amplitude
                    lower_offset = track_length - target_track_length
                else:
                    upper_amp = amplitude
                    upper_offset = track_length - target_track_length

            normals = 1j * dPdt / abs(dPdt)
            if not TrackGenerator.self_intersects(points[::2], normals[::2], margin / scale):
                break

        return scale * points, normals, scale * corner_radii

    # Self Intersection

    @staticmethod
    def _intersects(p, dp, q, dq):
        """Checks if the two line segments p->(p+dp) and q->(q+dq) intersect"""
        # map line segment p->(p+dp) to 0+0j->1+0j
        q = (q - p) / dp
        dq = dq / dp
        # handle case where dp and dq are parallel
        if dq.imag == 0:
            return q.imag == 0 and q.real < 1 and q.real + dq.real > 0
        else:
            # check if transformed line segment Q intersects with line 0,0 -> 1,0
            return q.imag * (q.imag + dq.imag) <= 0 and 0 < q.real - dq.real * q.imag / dq.imag < 1

    @staticmethod
    def _slf_intrsct_brute(edges):
        """
        Checks if any of the line segments in `edges` intersect by checking every
        pair of edges
        """
        for i, p_i in enumerate(edges):
            for p_j in edges[i + 1:]:
                # skip if the edges are adjacent
                if p_j[0] == p_i[1] or p_j[1] == p_i[0]:
                    continue

                if TrackGenerator._intersects(p_i[0], p_i[1] - p_i[0], p_j[0], p_j[1] - p_j[0]):
                    return True

        return False

    @staticmethod
    def _side(p, dp, edges):
        """
        checks whether the edge lies ontop (returns 0), to the left (returns 1), or
        to the right (returns -1) of the line that goes through p with slope dp
        """
        side0 = np.sign(((edges[:, 0] - p) / dp).imag)
        side1 = np.sign(((edges[:, 1] - p) / dp).imag)
        return np.sign(side0 + side1)

    @staticmethod
    def _slf_intrsct_recurse(edges):
        """
        Recursively checks if any of the line segments in `edges` intersect by
        splitting edges into two smaller non-intersecting sets.
        """
        if len(edges) <= 8:
            return TrackGenerator._slf_intrsct_brute(edges)

        center = sum(edges[:, 0] + edges[:, 1]) / (2 * len(edges))
        pivot = edges[len(edges) // 2][0]

        for n in range(32):
            side = TrackGenerator._side(center, pivot - center, edges)
            left = edges[side >= 0]
            right = edges[side <= 0]

            if abs(len(left) / len(edges) - 1 / 2) + abs(len(right) / len(edges) - 1 / 2) < 1 / 8:
                return (TrackGenerator._slf_intrsct_recurse(left)
                        or TrackGenerator._slf_intrsct_recurse(right))
            pivot = random.choice(edges)[0]

        # if we can't find a suitable split, then just brute force it
        return TrackGenerator._slf_intrsct_brute(edges)

    @staticmethod
    def _to_edges(points):
        """converts a cyclic sequence of points into a set of edges"""
        return np.column_stack((points, np.roll(points, -1)))

    @staticmethod
    def self_intersects(points, slopes, margin):
        """returns true if the track comes within margin of itself"""
        normals = 1j * slopes / abs(slopes)
        tmp1 = TrackGenerator._to_edges(points + margin * normals)
        tmp2 = TrackGenerator._to_edges(points - margin * normals)
        return (TrackGenerator._slf_intrsct_recurse(tmp1)
                or TrackGenerator._slf_intrsct_recurse(tmp2))

    # Starting Line Selection

    @staticmethod
    def _cyclic_smooth(indices, points, values, diameter):
        """returns the smoothed values of the points specified by indices"""
        distance_to_next = abs(np.append(np.diff(points), points[0] - points[-1]))

        smoothed_values = values[indices]
        for n, i in enumerate(indices):
            coef_sum = 1

            curr = (i if i != 0 else len(values)) - 1
            distance = distance_to_next[curr]
            while distance < diameter:
                coef = distance_to_next[curr] * math.sin(math.pi * distance / diameter)
                smoothed_values[n] += coef * values[curr]
                coef_sum += coef
                curr = (curr if curr != 0 else len(values)) - 1
                distance += distance_to_next[curr]

            smoothed_values[n] /= coef_sum

        return smoothed_values

    @staticmethod
    def pick_starting_point(
        positions, normals, corner_radii,
        starting_straight_length,
        downsample=2
    ):
        """
        Picks a suitable starting position, moves it to the beginning of the
        array, then translates and rotates the track such that the starting line
        faces towards 1,0 from 0,0

        Returns:
        A tuple containing the new position, normals, and corner_radii arrays

        Arguments
        starting_straight_length -- the starting line is set to the end of the
                                    stretch of length starting_straight_length
                                    with the smallest average curvature
        downsample               -- reduces the number of points by factor
                                    downsample to improve performance
        """
        # pick starting points
        smooth_diameter = 1.5 * starting_straight_length

        curvature = abs(1 / corner_radii[::downsample])
        # only check points with low curvature
        indices = np.argsort(curvature)[:len(curvature) // 8]
        start_index = (downsample * indices[np.argmin(TrackGenerator._cyclic_smooth(
            indices, positions, curvature, smooth_diameter))])

        positions = np.roll(positions, -start_index)
        normals = np.roll(normals, -start_index)
        corner_radii = np.roll(corner_radii, -start_index)

        # translate starting position to 0+0j
        positions -= positions[0]

        # rotate the starting position to face right
        rotation = 1j / normals[0]
        positions *= rotation
        normals *= rotation

        return positions, normals, corner_radii

    # Cone Placement

    @staticmethod
    def place_cones(
        positions, normals, corner_radii, min_corner_radius,
        min_cone_spacing, max_cone_spacing,
        track_width,
        cone_spacing_bias,
        start_offset,
        starting_cone_spacing
    ):
        """
        Generates starting, left and right cone locations from track path

        Returns:
        A tuple containing the left and right cone positions

        Arguments:
        positions, normals, corner_radii -- Points and attributes that define the
                                            path through the centre of the track
        min_cone_spacing  -- minimum distance between cones in metres
        max_cone_spacing  -- maximum distance between cones in metres
        track_width       -- track width in meters
        cone_spacing_bias -- controls the cone spacing on the outside of a turn
        start_offset      -- distance between the starting line and the car
        starting_cone_spacing -- distance between the two pairs of orange cones
                                marking the starting line
        """

        min_density = 1 / max_cone_spacing
        max_density = 1 / min_cone_spacing
        density_range = max_density - min_density

        c1 = density_range / 2 * ((1 - cone_spacing_bias) * min_corner_radius
                                  - (1 + cone_spacing_bias) * track_width / 2)
        c2 = density_range / 2 * ((1 + cone_spacing_bias) * min_corner_radius
                                  - (1 - cone_spacing_bias) * track_width / 2)

        def place(points, radii, side):
            distance_to_next = abs(np.append(np.diff(points), points[0] - points[-1]))
            distance_to_prev = np.roll(distance_to_next, 1)

            cone_density = min_density + side * c1 / radii + c2 / abs(radii)
            cone_density *= distance_to_prev

            # scale cone spacing to make the first and last cones match up
            modified_length = sum(cone_density)
            threshold = modified_length / round(modified_length)

            cones = [points[0]]
            current = 0
            for i, density in enumerate(cone_density[1:]):
                current += density
                if current >= threshold:
                    current -= threshold
                    cones.append(points[i])
            return np.array(cones)

        l_cones = place(positions + normals * track_width / 2, corner_radii - track_width / 2, 1)
        r_cones = place(positions - normals * track_width / 2, corner_radii + track_width / 2, -1)

        start_cones = np.array([l_cones[0], r_cones[0]])
        start_cones = np.append(start_cones + starting_cone_spacing / 2,
                                start_cones - starting_cone_spacing / 2)

        # put car start_offset behind the starting line
        car_pos = 0
        length_accum = 0
        while length_accum < start_offset:
            length_accum += abs(positions[car_pos - 1] - positions[car_pos])
            car_pos -= 1

        # translate car to 0+0j
        l_cones -= positions[car_pos]
        r_cones -= positions[car_pos]
        start_cones -= positions[car_pos]

        # rotate the car to face right
        rotation = 1j / normals[car_pos]
        l_cones *= rotation
        r_cones *= rotation
        start_cones *= rotation

        return start_cones, l_cones[1:], r_cones[1:]

    @staticmethod
    def write_to_csv(file_path, start_cones, l_cones, r_cones, overwrite=False):
        if not overwrite and exists(file_path):
            raise FileExistsError(f"'{file_path}' already exists")

        f = open(file_path, "w")
        f.write("tag,x,y,direction,x_variance,y_variance,xy_covariance\n")
        for cone in l_cones:
            f.write(f"blue,{cone.real:0.2f},{cone.imag:0.2f},0,0.01,0.01,0.0\n")
        for cone in r_cones:
            f.write(f"yellow,{cone.real:0.2f},{cone.imag:0.2f},0,0.01,0.01,0.0\n")

        for cone in start_cones:
            f.write(f"big_orange,{cone.real:0.2f},{cone.imag:0.2f},0,0.01,0.01,0.0\n")

        f.write("car_start,0.0,0.0,0,0.01,0.01,0.0\n")
        f.close()

    def set(self, properties):
        self.config = {**self.config, **properties}

        if 'seed' in properties:
            self.rng = random.Random(self.config['seed'])

    def __call__(self):
        margin = self.config['track_width'] / 2 + self.config['margin']
        if 'length' in self.config:
            path = TrackGenerator.generate_path_w_length(
                rng=self.rng,
                n_points=self.config['resolution'],
                min_corner_radius=self.config['min_corner_radius'],
                margin=margin,
                target_track_length=self.config['length'],
                rel_accuracy=self.config['rel_accuracy'],
                starting_amplitude=self.config['starting_amplitude']
            )
        elif 'max_frequency' in self.config:
            while True:
                path = TrackGenerator.generate_path_w_params(
                    rng=self.rng,
                    n_points=self.config['resolution'],
                    min_corner_radius=self.config['min_corner_radius'],
                    max_frequency=self.config['max_frequency'],
                    amplitude=self.config['amplitude']
                )
                if not (self.config['check_self_intersection']
                        and TrackGenerator.self_intersects(*path[:2], margin)):
                    break
        else:
            raise KeyError("missing one of required properties length or max_frequency")

        path = TrackGenerator.pick_starting_point(
            *path,
            starting_straight_length=self.config['starting_straight_length'],
            downsample=self.config['starting_straight_downsample']
        )

        return TrackGenerator.place_cones(
            *path, self.config['min_corner_radius'],
            min_cone_spacing=self.config['min_cone_spacing'],
            max_cone_spacing=self.config['max_cone_spacing'],
            track_width=self.config['track_width'],
            cone_spacing_bias=self.config['cone_spacing_bias'],
            start_offset=self.config['starting_straight_length'],
            starting_cone_spacing=self.config['starting_cone_spacing']
        )
