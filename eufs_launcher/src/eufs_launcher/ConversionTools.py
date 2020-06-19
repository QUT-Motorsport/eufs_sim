from PIL import Image
from PIL import ImageDraw
import math
from LauncherUtilities import (calculate_tangent_angle,
                               get_points_from_component_list,
                               compactify_points)
from random import randrange, uniform
from functools import reduce
import os
import rospkg
import rospy
import sys
import pandas as pd
from collections import OrderedDict
sys.path.insert(1, os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), 'tracks'))  # nopep8
from track_gen import Track
from TrackGenerator import (
        compactify_points, cone_start,
        CONE_INNER, CONE_OUTER,
        CONE_ORANGE, CONE_BIG_ORANGE,
        CONE_START, NOISE
)
from TrackGenerator import get_cone_function


# Here are all the track formats we care about:
# .launch (well, we actually want the model data, not the .launch, but we'll treat it as wanting the .launch)
#                (since the end user shouldn't have to care about the distinction)
# .png
# .csv
# raw_data ("xys",this will be hidden from the user as it is only used to convert into .pngs)
# raw_data may also be "comps", which is the output of TrackGenerator
class ConversionTools:
        def __init__():
                pass

        # Define colors for track gen and image reading
        noise_color = (0, 255, 255, 255)                 # cyan
        background_color = (255, 255, 255, 255)          # white
        inner_cone_color = (255, 0, 255, 255)            # magenta, for yellow cones (oops...)
        outer_cone_color = (0, 0, 255, 255)              # blue, for blue cones
        car_color = (0, 255, 0, 255)                     # green
        track_center_color = (0, 0, 0, 255)              # black
        track_inner_color = (255, 0, 0, 255)             # red
        track_outer_color = (255, 255, 0, 255)           # yellow
        orange_cone_color = (255, 165, 0, 255)           # orange, for orange cones
        big_orange_cone_color = (127, 80, 0, 255)        # dark orange, for big orange cones
        lap_counter_color = (202, 44, 146, 255)          # fuschia, for lap counters (skidpad)

        double_orange_cone_color = (200, 100, 0, 255)    # for double-orange-cones

        # Other various retained parameters
        link_num = -1
        TRACKIMG_VERSION_NUM = 2

        # Metadata corner variables
        TOP_LEFT = "Top Left"
        BOTTOM_LEFT = "Bottom Left"
        TOP_RIGHT = "Top Right"
        BOTTOM_RIGHT = "Bottom Right"

        # Double cone closeness, how close do we place the doubled-up starting cones?
        DOUBLE_CONE_CLOSENESS = 0.7

        # Magic numbers to make the 3n -> 4 + n byte conversion work
        # These work but are likely not optimal
        fbtf_min = 1.5
        fbtf_max = 3

        #########################################################
        #              Handle Track Image Metadata              #
        #########################################################

        @staticmethod
        def get_metadata_pixel(x, y, corner, pixels, size):
                """
                Returns the metadata pixel (x,y,corner)'s value.

                x: x position within the corner
                y: y position within the corner
                corner: which corner of the image we are in
                pixels: pixels of the image
                size: size of the image
                """

                x_out, y_out = ConversionTools.get_metadata_pixel_location(x, y, corner, size)
                return pixels[x_out, y_out]

        @staticmethod
        def get_metadata_pixel_location(x, y, corner, size):
                """Returns the metadata pixel (x,y,corner)'s location on the actual image."""
                width = size[0]
                height = size[1]
                if corner == ConversionTools.TOP_LEFT:
                        return (x, y)
                elif corner == ConversionTools.BOTTOM_LEFT:
                        return (x, height - 6 + y)
                elif corner == ConversionTools.TOP_RIGHT:
                        return (width - 6 + x, y)
                elif corner == ConversionTools.BOTTOM_RIGHT:
                        return (width - 6 + x, height - 6 + y)
                else:
                        rospy.logerr("Error, not a valid corner!  Typo?: " + corner)
                        return None

        @staticmethod
        def get_raw_metadata(pixel_value):
                """
                This function converts metadata as outlined in the
                specification for Track Images on the team wiki
                It assumes that handling of the cases (255,255,255,255) and (r,g,b,0)
                are done outside this function.

                pixel_value: the (r,g,b,a) value of the pixel

                """
                (r, g, b, a) = pixel_value
                return a - 1 + (b - 1) * 254 + (g - 1) * 254**2 + (r - 1) * 254**3

        @staticmethod
        def unget_raw_metadata(metadata):
                """Undoes the process of get_raw_metadata()"""
                a = metadata % 254
                metadata = (metadata - a) // 254
                b = metadata % 254
                metadata = (metadata - b) // 254
                g = metadata % 254
                r = (metadata - g) // 254

                return (r + 1, g + 1, b + 1, a + 1)

        @staticmethod
        def convert_scale_metadata(pixel_values):
                """
                This function converts the data obtained from
                scale metadata pixels into actual scale information
                Output range is from 0.0001 to 100.
                """

                primary_pixel = pixel_values[0]
                secondary_pixel = pixel_values[1]

                # Check for default pixel values
                if (primary_pixel == (255, 255, 255, 255) and
                   secondary_pixel == (255, 255, 255, 255)):
                        return 1

                metadata = ConversionTools.get_raw_metadata(primary_pixel)

                # Want to linearly transform the metadata, a range from 0 to 254**4-1,
                # to the range 0.0001 to 100
                return metadata / (254**4 - 1.0) * (100 - 0.0001) + 0.0001

        @staticmethod
        def deconvert_scale_metadata(data):
                """
                This function converts a raw_ scale value into a
                list of metadata pixels needed to replicate it.
                First in list is the primary metadata pixel, second in list is the secondary
                """

                metadata = int((data - 0.0001) / (100 - 0.0001) * (254**4 - 1))
                primary_pixel = ConversionTools.unget_raw_metadata(metadata)
                secondary_pixel = (255, 255, 255, 255)
                return [primary_pixel, secondary_pixel]

        @staticmethod
        def convert_version_metadata(pixel_values):
                """
                This function converts the data obtained from
                scale metadata pixels into actual scale information
                Output range is from 0 to 254**4-1
                """
                primary_pixel = pixel_values[0]
                if primary_pixel == (255, 255, 255, 255):
                    return 0
                metadata = ConversionTools.get_raw_metadata(primary_pixel)
                return metadata

        @staticmethod
        def deconvert_version_metadata(data):
                """This function is the reverse transformation as convert_version_metadata"""
                return [ConversionTools.unget_raw_metadata(data)]

        #########################################################
        #                Main Conversion Method                 #
        #########################################################

        @staticmethod
        def convert(cfrom, cto, which_file, params={}, conversion_suffix="", override_name=None):
                """
                Will convert which_file of filetype cfrom to filetype cto with
                filename which_file+conversion_suffix

                cfrom:      Type to convert from (xys, png, launch, csv) [should be a string]
                cto:        Type to convert to   (png, launch, csv)      [should be a string]

                which_file: The file to be converted - should be a full filepath.

                params:     Additional parameters that may be necessary.
                            These will depend on conversion type, so check the docstrings of
                            the specific desired conversion function for full information.

                conversion_suffix: A suffix appended to the filename of the output file.

                If converted to an image, it will return that image in case the function caller
                desires to do any displaying of the result (otherwise it returns None)
                """

                if cfrom == "xys" and cto == "png":
                    return ConversionTools.xys_to_png(
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                elif cfrom == "comps" and cto == "png":
                    return ConversionTools.comps_to_png(
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                elif cfrom == "comps" and cto == "csv":
                    return ConversionTools.comps_to_csv(
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                elif cfrom == "png" and cto == "launch":
                    return ConversionTools.png_to_launch(
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                elif cfrom == "png" and cto == "csv":
                    ConversionTools.png_to_launch(
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                    new_file_array = which_file.split("/")
                    new_file_array[-2] = "launch"
                    which_file = "/".join(new_file_array)
                    return ConversionTools.launch_to_csv(
                             which_file[:-4]+conversion_suffix+".launch",
                             params,
                             conversion_suffix="",
                             override_name=override_name
                    )
                elif cfrom == "launch" and cto == "csv":
                    return ConversionTools.launch_to_csv(
                             which_file,
                             params,
                             conversion_suffix,
                             override_name
                    )
                elif cfrom == "launch" and cto == "png":
                    ConversionTools.launch_to_csv(
                             which_file,
                             params,
                             conversion_suffix,
                             override_name
                    )
                    new_file_array = which_file.split("/")
                    new_file_array[-2] = "tracks"
                    which_file = "/".join(new_file_array)
                    return ConversionTools.csv_to_png(
                             which_file[:-7]+conversion_suffix+".csv",
                             params,
                             conversion_suffix="",
                             override_name=override_name
                    )
                elif cfrom == "csv" and cto == "launch":
                    return ConversionTools.csv_to_launch(
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                elif cfrom == "csv" and cto == "png":
                    return ConversionTools.csv_to_png(
                             which_file,
                             params,
                             conversion_suffix,
                             override_name
                    )
                elif cto == "ALL" or cto == "all":
                    # If something tries to convert to itself it just gets ignored
                    ConversionTools.convert(
                            cfrom,
                            "launch",
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                    ConversionTools.convert(
                            cfrom,
                            "csv",
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                    return ConversionTools.convert(
                            cfrom,
                            "png",
                            which_file,
                            params,
                            conversion_suffix,
                            override_name
                    )
                return None

        @staticmethod
        def comps_to_csv(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts raw track generator output to csv.

                which_file:            Output filename
                params["track data"]:  The track generator data
                conversion_suffix:     Additional suffix to append to the output filename.

                Should be called within a GeneratorContext context manager.
                """

                GENERATED_FILENAME = which_file + conversion_suffix
                if override_name is not None:
                    GENERATED_FILENAME = override_name

                # Unpack
                (components, twidth, theight) = params["track data"]

                xys = compactify_points([
                        (int(x[0]), int(x[1])) for x
                        in get_points_from_component_list(components)
                ])

                # We want to calculate direction of car position
                sx = xys[0][0]
                sy = xys[0][1]
                ex = xys[1][0]
                ey = xys[1][1]

                # So we calculate the angle that the car is facing (the yaw)
                angle = math.atan2(ey - sy, ex - sx)
                if angle < 0:
                        # Angle is on range [-pi,pi] but we want [0,2pi]
                        # So if it is less than 0, pop it onto the correct range.
                        angle += 2 * math.pi

                # Now we get the car position
                car_x, car_y = (sx, sy)

                # And now the cone locations
                cone_locs = []
                for idx, tup in enumerate(components):
                        (name, points) = tup
                        if idx == 0:
                                cone_locs.extend(cone_start(points))
                        else:
                                cone_locs.extend(
                                        get_cone_function(name)(points, prev_points=cone_locs)
                                )

                # Get the right csv tags
                def name_from_color(c):
                    if c == CONE_START or c == CONE_BIG_ORANGE:
                        return "big_orange"
                    elif c == CONE_ORANGE:
                        return "orange"
                    elif c == CONE_OUTER:
                        return "yellow"
                    elif c == CONE_INNER:
                        return "blue"
                    elif c == NOISE:
                        return "inactive_noise"
                    else:
                        return "ERROR"

                # Add 2 cones at the cone start
                to_remove = []
                for cone in cone_locs:
                    px, py, color = cone
                    if color == CONE_START:
                        direction_vector = (
                                math.cos(angle) * 0.5 * ConversionTools.DOUBLE_CONE_CLOSENESS,
                                math.sin(angle) * 0.5 * ConversionTools.DOUBLE_CONE_CLOSENESS
                        )
                        new_cone1 = (
                            cone[0] - direction_vector[0],
                            cone[1] - direction_vector[1],
                            CONE_BIG_ORANGE
                        )
                        new_cone2 = (
                            cone[0] + direction_vector[0],
                            cone[1] + direction_vector[1],
                            CONE_BIG_ORANGE
                        )
                        cone_locs.append(new_cone1)
                        cone_locs.append(new_cone2)
                        to_remove.append(cone)
                [cone_locs.remove(x) for x in to_remove]

                # Add noise pixels
                im_to_display = ConversionTools.comps_to_png(GENERATED_FILENAME, params)
                pixels = im_to_display.load()
                for i in range(im_to_display.size[0]):
                        for j in range(im_to_display.size[1]):
                                p = pixels[i, j]
                                if p == ConversionTools.noise_color:
                                    cone_locs.append((1.0 * i, 1.0 * j, NOISE))

                # Save it
                output = OrderedDict([
                    ("tag", [name_from_color(color) for px, py, color in cone_locs] +
                        ["car_start"]),
                    ("x", [px for px, py, color in cone_locs] + [car_x]),
                    ("y", [py for px, py, color in cone_locs] + [car_y]),
                    ("direction", [0 for x in cone_locs] + [angle]),
                    ("x_variance", [0 for x in cone_locs] + [0]),
                    ("y_variance", [0 for x in cone_locs] + [0]),
                    ("xy_covariance", [0 for x in cone_locs] + [0])
                ])
                df = pd.DataFrame(output, columns=output.keys())
                df.to_csv(
                    os.path.join(
                        rospkg.RosPack().get_path('eufs_gazebo'),
                        'tracks/'+GENERATED_FILENAME+'.csv'
                    ),
                    index=False
                )

                return im_to_display

        @staticmethod
        def comps_to_png(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts raw track generator output to png.

                which_file:            Output filename
                params["track data"]:  The track generator data
                conversion_suffix:     Additional suffix to append to the output filename.

                Should be called within a GeneratorContext context manager.
                """

                GENERATED_FILENAME = which_file + conversion_suffix
                if override_name is not None:
                    GENERATED_FILENAME = override_name

                # Unpack
                (components, twidth, theight) = params["track data"]

                xys = compactify_points([
                        (int(x[0]), int(x[1])) for x
                        in get_points_from_component_list(components)
                ])

                # Create image to hold data
                im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0))
                draw = ImageDraw.Draw(im)

                # Draw background:
                bounding_poly = [
                                 (0, 0),
                                 (twidth, 0),
                                 (twidth, theight),
                                 (0, theight),
                                 (0, 0)
                ]
                draw.polygon(bounding_poly, fill='white')

                # Draw thick track with colored layers
                draw.line(xys, fill=ConversionTools.track_outer_color, width=5)
                draw.line(xys, fill=ConversionTools.track_inner_color, width=3)
                draw.line(xys, fill=ConversionTools.track_center_color)

                pixels = im.load()

                # We want to calculate direction of car position
                sx = xys[0][0]
                sy = xys[0][1]
                ex = xys[1][0]
                ey = xys[1][1]

                # So we calculate the angle that the car is facing (the yaw)
                angle = math.atan2(ey - sy, ex - sx)
                if angle < 0:
                        # Angle is on range [-pi,pi] but we want [0,2pi]
                        # So if it is less than 0, pop it onto the correct range.
                        angle += 2 * math.pi

                # We convert angles to range [1,255] rather than [0,255]
                # As if it is 0 then, since it is stored as an alpha value,
                # optimization techniques for storage may be used by the png
                # file format to set the whole pixel to some standard value,
                # such as (0,0,0,0), which is decidedly not what we want as
                # the rgb values still matter.
                yaw_pixel_value = int(angle / (2 * math.pi) * 254 + 1)
                if yaw_pixel_value > 255:
                    yaw_pixel_value = 255
                if yaw_pixel_value < 1:
                    yaw_pixel_value = 1

                # Draw car
                color_for_car = ConversionTools.car_color[:3]+(yaw_pixel_value,)
                car_pos_to_place = (sx, sy)
                draw.line([car_pos_to_place, car_pos_to_place], fill=color_for_car)

                # Now we want to make all pixels bordering the track become magenta (255,0,255) -
                # this will be our 'cone' color
                # To find pixel boardering track, simply find white pixel adjacent to
                # a non-white non-magenta pixle
                # We will also want to make it such that cones are about 4-6
                # away from eachother euclideanly

                def is_track(c):
                        return (c == ConversionTools.track_outer_color or
                                c == ConversionTools.track_inner_color or
                                c == ConversionTools.track_center_color)

                cone_locs = []
                for idx, tup in enumerate(components):
                        (name, points) = tup
                        if idx == 0:
                                cone_locs.extend(cone_start(points))
                        else:
                                cone_locs.extend(
                                        get_cone_function(name)(points, prev_points=cone_locs)
                                )

                for px, py, color in cone_locs:
                        if color is CONE_INNER:
                                true_color = ConversionTools.inner_cone_color
                        elif color is CONE_OUTER:
                                true_color = ConversionTools.outer_cone_color
                        elif color is CONE_ORANGE:
                                true_color = ConversionTools.orange_cone_color
                        elif color is CONE_START:
                                true_color = (
                                        # Double cones need angle information
                                        ConversionTools.double_orange_cone_color[:3] + (yaw_pixel_value,)
                                )

                        # Only allow orange cones to be placed on the track
                        if (not is_track(pixels[int(px), int(py)]) or color is CONE_ORANGE):
                                pixels[int(px), int(py)] = true_color

                # Finally, we just need to place noise.
                # At maximal noise, the track should be about 1% covered.

                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                # Don't add noise in margin
                                if i < 5 or j < 5 or i >= im.size[0] - 5 or j >= im.size[0] - 5:
                                        continue
                                if pixels[i, j] == ConversionTools.background_color:
                                        # Only 1% should be covered with noise
                                        if uniform(0, 100) < 1:
                                                pixels[i, j] = ConversionTools.noise_color

                # Add margins (as specified by the file format)
                margin = 5
                im2 = Image.new(
                                'RGBA',
                                (twidth + 2 * margin, theight + 2 * margin),
                                (255, 255, 255, 255)
                )
                pixels2 = im2.load()
                for x in range(im.size[0]):
                        for y in range(im.size[1]):
                                pixels2[x + margin, y + margin] = pixels[x, y]

                # And tag it with the version number
                loc = ConversionTools.get_metadata_pixel_location(
                                            4,
                                            4,
                                            ConversionTools.BOTTOM_RIGHT,
                                            im2.size
                )
                pixels2[loc[0], loc[1]] = ConversionTools.deconvert_version_metadata(
                                             ConversionTools.TRACKIMG_VERSION_NUM
                )[0]

                im2.save(
                        os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),
                                     'randgen_imgs/'+GENERATED_FILENAME+'.png')
                )
                return im

        @staticmethod
        def xys_to_png(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts xys format to png.

                which_file:            Output filename
                params["point list"]:  Tuple of: list of points, image width, image height
                params["track width"]: distance from center of track to cones.
                                        is optional - if not provided, should be called
                                        within a GeneratorContext context manager.
                conversion_suffix:     Additional suffix to append to the output filename.

                Should be called within a GeneratorContext context manager.
                """

                GENERATED_FILENAME = which_file + conversion_suffix
                if override_name is not None:
                    GENERATED_FILENAME = override_name

                # Unpack
                (xys, twidth, theight) = params["point list"]

                # Create image to hold data
                im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0))
                draw = ImageDraw.Draw(im)

                # Draw background:
                bounding_poly = [
                                 (0, 0),
                                 (twidth, 0),
                                 (twidth, theight),
                                 (0, theight),
                                 (0, 0)
                ]
                draw.polygon(bounding_poly, fill='white')

                # Draw thick track with colored layers
                draw.line(xys, fill=ConversionTools.track_outer_color, width=5)
                draw.line(xys, fill=ConversionTools.track_inner_color, width=3)
                draw.line(xys, fill=ConversionTools.track_center_color)

                pixels = im.load()

                # We want to calculate direction of car position
                sx = xys[0][0]
                sy = xys[0][1]
                ex = xys[1][0]
                ey = xys[1][1]

                # So we calculate the angle that the car is facing (the yaw)
                angle = math.atan2(ey - sy, ex - sx)
                if angle < 0:
                        # Angle is on range [-pi,pi] but we want [0,2pi]
                        # So if it is less than 0, pop it onto the correct range.
                        angle += 2 * math.pi

                # We convert angles to range [1,255] rather than [0,255]
                # As if it is 0 then, since it is stored as an alpha value,
                # optimization techniques for storage may be used by the png
                # file format to set the whole pixel to some standard value,
                # such as (0,0,0,0), which is decidedly not what we want as
                # the rgb values still matter.
                yaw_pixel_value = int(angle / (2 * math.pi) * 254 + 1)
                if yaw_pixel_value > 255:
                    yaw_pixel_value = 255
                if yaw_pixel_value < 1:
                    yaw_pixel_value = 1

                # Draw car
                color_for_car = ConversionTools.car_color[:3]+(yaw_pixel_value,)
                draw.line([xys[0], xys[0]], fill=color_for_car)

                # Now we want to make all pixels bordering the track become magenta (255,0,255) -
                # this will be our 'cone' color
                # To find pixel boardering track, simply find white pixel adjacent to
                # a non-white non-magenta pixle
                # We will also want to make it such that cones are about 4-6
                # away from eachother euclideanly

                def is_track(c):
                        return (c == ConversionTools.track_outer_color or
                                c == ConversionTools.track_inner_color or
                                c == ConversionTools.track_center_color)

                if "track width" in params:
                        cone_locs = cone_start(
                                xys,
                                width=params["track width"]
                        )
                else:
                        cone_locs = cone_start(xys)

                for px, py, color in cone_locs:
                        if color is CONE_INNER:
                                true_color = ConversionTools.inner_cone_color
                        elif color is CONE_OUTER:
                                true_color = ConversionTools.outer_cone_color
                        elif color is CONE_ORANGE:
                                true_color = ConversionTools.orange_cone_color
                        elif color is CONE_START:
                                true_color = (
                                        # Double cones need angle information
                                        ConversionTools.double_orange_cone_color[:3] + (yaw_pixel_value,)
                                )
                        if (not is_track(pixels[int(px), int(py)])):
                                pixels[int(px), int(py)] = true_color

                # Finally, we just need to place noise.
                # At maximal noise, the track should be about 1% covered.

                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                # Don't add noise in margin
                                if i < 5 or j < 5 or i >= im.size[0] - 5 or j >= im.size[0] - 5:
                                        continue
                                if pixels[i, j] == ConversionTools.background_color:
                                        # Only 1% should be covered with noise
                                        if uniform(0, 100) < 1:
                                                pixels[i, j] = ConversionTools.noise_color

                # Add margins (as specified by the file format)
                margin = 5
                im2 = Image.new(
                                'RGBA',
                                (twidth + 2 * margin, theight + 2 * margin),
                                (255, 255, 255, 255)
                )
                pixels2 = im2.load()
                for x in range(im.size[0]):
                        for y in range(im.size[1]):
                                pixels2[x + margin, y + margin] = pixels[x, y]

                # And tag it with the version number
                loc = ConversionTools.get_metadata_pixel_location(
                                            4,
                                            4,
                                            ConversionTools.BOTTOM_RIGHT,
                                            im2.size
                )
                pixels2[loc[0], loc[1]] = ConversionTools.deconvert_version_metadata(
                                             ConversionTools.TRACKIMG_VERSION_NUM
                )[0]

                im2.save(
                        os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),
                                     'randgen_imgs/'+GENERATED_FILENAME+'.png')
                )
                return im

        @staticmethod
        def png_to_launch(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts a .png to a .launch with .world, model.sdf, model.config files.

                which_file:        The name of the png file to convert
                                   example: rand.png

                params["noise"]:   Percentage of noise to activate

                conversion_suffix: Something to append to the output filename
                                   So if it is "foo", rand.png becomes randfoo.launch.

                This is a multistep process - we need:
                        to put %FILENAME%.launch in eufs_gazebo/launch
                        to put %FILENAME%.world in eufs_gazebo/world
                        to put %FILENAME%/model.config and %FILENAME%/model.sdf
                                 in eufs_description/models

                Our template files are stored in eufs_launcher/resource as:
                        randgen_launch_template
                        randgen_world_template
                        randgen_model_template/model.config
                        randgen_model_template/model.sdf
                """
                GENERATED_FILENAME = which_file.split('/')[-1][:-4]+conversion_suffix
                if override_name is not None:
                    GENERATED_FILENAME = override_name
                im = Image.open(which_file)
                noise_level = params["noise"] if "noise" in params else 0
                pixels = im.load()

                # Let's get the scale metadata from the png:
                # scale_data will represent how big a pixel is in meters^2
                scale_data = ConversionTools.convert_scale_metadata([
                        ConversionTools.get_metadata_pixel(
                                                           0,
                                                           0,
                                                           ConversionTools.TOP_LEFT,
                                                           pixels,
                                                           im.size
                        ),
                        ConversionTools.get_metadata_pixel(
                                                           1,
                                                           0,
                                                           ConversionTools.TOP_LEFT,
                                                           pixels,
                                                           im.size
                        )
                ])

                # Let's also get the version number - we don't need it,
                # but in the future if breaking changes are made to the
                # Track Image specification then it will become important.
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  4,
                                                                  4,
                                                                  ConversionTools.BOTTOM_RIGHT,
                                                                  im.size
                )
                version_number = ConversionTools.convert_version_metadata([pixels[loc[0], loc[1]]])

                # Let's start filling in the .launch template:
                launch_template_file = os.path.join(
                                                    rospkg.RosPack().get_path('eufs_launcher'),
                                                    'resource/randgen_launch_template'
                )
                launch_template = open(launch_template_file, "r")

                # .launches need to point to .worlds and model files of the same name,
                # so here we are pasting in copies of the relevant filename.
                launch_merged = "".join(launch_template)
                launch_merged = GENERATED_FILENAME.join(launch_merged.split("%FILLNAME%"))

                # These are helper functions to convert pixel data, such as coordinates,
                # into raw numerical data for the .launch file.
                def x_coord_transform(x):
                        return x - 50

                def y_coord_transform(y):
                        return y - 50

                def is_car_color(x):
                        return x[:-1] == ConversionTools.car_color[:-1]

                def rotation_transform(x):
                        return 2 * math.pi * ((x - 1) / 254.0)

                # Find the car's position
                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                p = pixels[i, j]
                                if is_car_color(p):
                                        # Get x data
                                        x_pos = x_coord_transform(i * scale_data)
                                        split_x_data = launch_merged.split("%PLACEX%")
                                        launch_merged = str(x_pos).join(split_x_data)

                                        # Get y data
                                        y_pos = y_coord_transform(j * scale_data)
                                        split_y_data = launch_merged.split("%PLACEY%")
                                        launch_merged = str(y_pos).join(split_y_data)

                                        # Get rotation data
                                        rot_info = rotation_transform(p[3])
                                        split_rot_data = launch_merged.split("%PLACEROTATION%")
                                        launch_merged = str(rot_info).join(split_rot_data)

                # Here we write out to our launch file.
                launch_out_filename = 'launch/'+GENERATED_FILENAME+".launch"
                launch_out_filepath = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_gazebo'),
                                                   launch_out_filename
                )
                launch_out = open(launch_out_filepath, "w")
                launch_out.write(launch_merged)
                launch_out.close()

                # And now we start the template for the .world:
                world_template_filepath = os.path.join(
                                              rospkg.RosPack().get_path('eufs_launcher'),
                                              'resource/randgen_world_template'
                )
                world_template = open(world_template_filepath, "r")

                # The world file needs to point to the correct model folder,
                # which conveniently has the same name as the world file itself.
                world_merged = "".join(world_template)
                world_merged = GENERATED_FILENAME.join(world_merged.split("%FILLNAME%"))

                # And now we write out.
                world_out_filepath = os.path.join(
                                                  rospkg.RosPack().get_path('eufs_gazebo'),
                                                  'worlds',
                                                  GENERATED_FILENAME+".world"
                )
                world_out = open(world_out_filepath, "w")
                world_out.write(world_merged)
                world_out.close()

                # And now we work on making the model folder:
                # First we create the folder itself
                folder_path = os.path.join(
                                           rospkg.RosPack().get_path('eufs_description'),
                                           'models',
                                           GENERATED_FILENAME
                )

                # If the folder does not exist, which is usual, create it.
                # If the folder does exist, it gets automatically overridden
                # by the rest of this function - this behavior is intended.
                if not os.path.exists(folder_path):
                        os.mkdir(folder_path)

                # Now let's do the .config
                config_template_filepath = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_launcher'),
                                                   'resource/randgen_model_template/model.config'
                )
                config_template = open(config_template_filepath, "r")

                # Let the config file know the name of the track it represents
                config_merged = "".join(config_template)
                config_merged = GENERATED_FILENAME.join(config_merged.split("%FILLNAME%"))

                # Write out the config data
                config_out_filepath = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_description'),
                                                   'models',
                                                   GENERATED_FILENAME,
                                                   "model.config"
                )
                config_out = open(config_out_filepath, "w")
                config_out.write(config_merged)
                config_out.close()

                # Now we create the .sdf
                # This is fairly intensive
                sdf_template_filepath = os.path.join(
                                                     rospkg.RosPack().get_path('eufs_launcher'),
                                                     'resource/randgen_model_template/model.sdf'
                )
                sdf_template = open(sdf_template_filepath, "r")
                sdf_merged = "".join(sdf_template)
                sdf_split_again = sdf_merged.split("$===$")

                # sdf_split_again list contents:
                #        0: Main body of sdf file
                #        1: Outline of noise mesh visual data
                #        2: Outline of noise mesh collision data
                #        3: Noisecube collision data, meant for noise as
                #            a low-complexity collision to prevent falling out the world
                #        4: Outline of noise mesh visual data for innactive noise

                # Here we break up sdf_split_again into its constituent parts
                # At the end of this process we will have:
                #        sdf_main:                  The global template for the sdf
                #        sdf_model:                 The template for cone and noise models
                #        sdf_model_with_collisions: Similar to sdf_model, but with collision data.
                #
                # And the corresponding sdf_ghost_model and sdf_ghost_model_with_collisions
                # which store inactive (hidden) models.
                sdf_main = sdf_split_again[0]
                sdf_split_along_collisions = sdf_split_again[6].split("%FILLCOLLISION%")
                sdf_split_along_ghost_collisions = sdf_split_again[6].split("%FILLCOLLISION%")
                sdf_model = sdf_split_again[3].join(sdf_split_along_collisions)
                sdf_model_with_collisions = sdf_split_again[2].join(sdf_split_along_collisions)
                sdf_ghost_model = sdf_split_again[3].join(sdf_split_along_ghost_collisions)
                sdf_ghost_model_with_collisions = sdf_split_again[2].join(sdf_split_along_ghost_collisions)

                # Let the sdf file know which launch file it represents.
                sdf_main = GENERATED_FILENAME.join(sdf_main.split("%FILLNAME%"))

                # Set up a helper function for calculating model data for cones
                collision_template = sdf_model_with_collisions.split("%MODELNAME%")

                def join_cone_model_data(cone_type):
                        flip_cones = cone_type.split("_")
                        if len(flip_cones) == 1:
                                name = flip_cones[0]
                        else:
                                name = flip_cones[1] + "_" + flip_cones[0]
                        cone_string = "model://models/"+name
                        return cone_string.join(collision_template)

                def setup_covariance(x, y, xy):
                        covariance_template = sdf_split_again[5]
                        output = str(x).join(covariance_template.split("%XCOV%"))
                        output = str(y).join(output.split("%YCOV%"))
                        output = str(xy).join(output.split("%XYCOV%"))
                        return output

                # Calculate model data for cones
                sdf_blue_cone_model = join_cone_model_data("cone_blue")
                sdf_yellow_cone_model = join_cone_model_data("cone_yellow")
                sdf_orange_cone_model = join_cone_model_data("cone")
                sdf_big_orange_cone_model = join_cone_model_data("cone_big")

                # Now let's load in the noise priorities
                noise_priority_file = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_launcher'),
                                                   'resource/noiseFiles.txt'
                )
                noise_files = open(noise_priority_file, "r")
                noise_files = ("".join(noise_files)).split("$===$")[1].strip().split("\n")
                noise_weightings_ = [line.split("|") for line in noise_files]
                noise_weightings = [(float(line[0]), line[1]) for line in noise_weightings_]

                # This function will choose a noise model according to weightings
                # given in the resource/noiseFiles.txt file.
                def get_random_noise_model():
                        randval = uniform(0, 100)
                        for a in noise_weightings:
                                if a[0] > randval:
                                        return a[1]
                        return "model://eufs_description/meshes/NoiseCube.dae"

                # Let's place all the models!
                # We'll keep track of how many we've placed
                # so that we can give each a unique name.
                ConversionTools.link_num = -1

                def put_model_at_position(mod, x, y, modtype, x_cov=0.01, y_cov=0.01, xy_cov=0):
                        """
                        mod: model template to be placed
                        x,y: x and y positions for mod

                        returns model template with x,y, and link_num inserted.
                        """
                        ConversionTools.link_num += 1
                        link_str = str(ConversionTools.link_num)
                        x_str = str(x_coord_transform(x))
                        y_str = str(y_coord_transform(y))
                        mod_with_y = y_str.join(mod.split("%PLACEY%"))
                        mod_with_x = x_str.join(mod_with_y.split("%PLACEX%"))
                        mod_with_link = link_str.join(mod_with_x.split("%LINKNUM%"))
                        mod_with_link2 = modtype.join(mod_with_link.split("%LINKTYPE%"))
                        x_cov_str = str(x_cov)
                        y_cov_str = str(y_cov)
                        xy_cov_str = str(xy_cov)
                        rospy.logerr(x_cov_str)
                        mod_with_cov = setup_covariance(x_cov_str, y_cov_str, xy_cov_str).join(
                                mod_with_link2.split("%FILLCOVARIANCE%")
                        )
                        rospy.logerr(mod_with_cov)
                        return mod_with_cov

                sdf_allmodels = ""

                def expand_allmodels(allmods, mod, modtype, x, y):
                        """
                        Takes in a model and a pixel location,
                        converts the pixel location to a raw location,
                        and places the model inside sdf_allmodels
                        """
                        mod_at_p = put_model_at_position(mod, x * scale_data, y * scale_data, modtype)
                        return allmods + "\n" + mod_at_p

                def doubly_expand_allmodels(allmods, mod, modtype, x, y, yaw):
                        """
                        Takes in a model and a pixel location,
                        converts the pixel location to a raw location,
                        and places the model inside sdf_allmodels

                        Unlike expand_allmodels, it also takes in an angle and will place
                        a second model near the first but slightly offset in the direction
                        of the yaw angle parameter.

                        Used to place the double cones in the beginning.
                        """

                        # Determines how close the two models are.  Smaller means closer.
                        closeness_parameter = scale_data * ConversionTools.DOUBLE_CONE_CLOSENESS

                        direction_vector = (
                                math.cos(yaw) * 0.5 * closeness_parameter,
                                math.sin(yaw) * 0.5 * closeness_parameter
                        )

                        mod_at_p1 = put_model_at_position(
                                mod,
                                x * scale_data + direction_vector[0],
                                y * scale_data + direction_vector[1],
                                modtype
                        )

                        mod_at_p2 = put_model_at_position(
                                mod,
                                x * scale_data - direction_vector[0],
                                y * scale_data - direction_vector[1],
                                modtype
                        )

                        return allmods + "\n" + mod_at_p1 + "\n" + mod_at_p2

                def get_random_noise_template(mod_with_collisions):
                        """
                        Gets a template for an arbitrary noise model
                        """
                        return get_random_noise_model().join(
                                      mod_with_collisions.split("%MODELNAME%")
                        )

                # Add all models into a template
                color_to_model = {
                        ConversionTools.inner_cone_color[:3]: sdf_yellow_cone_model,
                        ConversionTools.outer_cone_color[:3]: sdf_blue_cone_model,
                        ConversionTools.orange_cone_color[:3]: sdf_orange_cone_model,
                        ConversionTools.big_orange_cone_color[:3]: sdf_big_orange_cone_model
                }
                color_to_name = {
                        ConversionTools.inner_cone_color[:3]: "yellow_cone",
                        ConversionTools.outer_cone_color[:3]: "blue_cone",
                        ConversionTools.orange_cone_color[:3]: "orange_cone",
                        ConversionTools.big_orange_cone_color[:3]: "big_cone"
                }
                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                p = pixels[i, j]
                                if p == ConversionTools.noise_color:
                                        if uniform(0, 1) < noise_level:
                                                # Noise should be placed
                                                sdf_noisemodel = get_random_noise_template(
                                                                   sdf_model_with_collisions
                                                )
                                                sdf_allmodels = expand_allmodels(
                                                                       sdf_allmodels,
                                                                       sdf_noisemodel,
                                                                       "active_noise",
                                                                       i,
                                                                       j
                                                )
                                        else:
                                                # Noise exists but not active
                                                sdf_noisemodel = get_random_noise_template(
                                                                   sdf_ghost_model_with_collisions
                                                )
                                                sdf_allmodels = expand_allmodels(
                                                                       sdf_allmodels,
                                                                       sdf_noisemodel,
                                                                       "inactive_noise",
                                                                       i,
                                                                       j,
                                                )
                                elif p[:3] in color_to_model:
                                        # Normal cones.
                                        sdf_allmodels = expand_allmodels(
                                                            sdf_allmodels,
                                                            color_to_model[p[:3]],
                                                            color_to_name[p[:3]],
                                                            i,
                                                            j
                                        )
                                elif p[:3] == ConversionTools.double_orange_cone_color[:3]:
                                        # Double cones!  Need to make use of yaw values.

                                        # Convert from alpha value to angle
                                        model_yaw = ((p[3] - 1) / 254) * (2 * math.pi)

                                        sdf_allmodels = doubly_expand_allmodels(
                                                            sdf_allmodels,
                                                            sdf_big_orange_cone_model,
                                                            "big_cone",
                                                            i,
                                                            j,
                                                            model_yaw
                                        )
                                elif p[:3] == ConversionTools.lap_counter_color[:3]:
                                        # Lap counter!
                                        sdf_allmodels = expand_allmodels(
                                            sdf_allmodels,
                                            join_cone_model_data("lap_counter;" + str(255 - p[3])),
                                            "lap_counter;" + str(255 - p[3]),
                                            i,
                                            j
                                        )

                # Splice the sdf file back together.
                sdf_main = sdf_allmodels.join(sdf_main.split("%FILLDATA%"))

                # Write it out.
                sdf_out_filepath = os.path.join(
                                                rospkg.RosPack().get_path('eufs_description'),
                                                'models',
                                                GENERATED_FILENAME,
                                                "model.sdf"
                )
                sdf_out = open(sdf_out_filepath, "w")
                sdf_out.write(sdf_main)
                sdf_out.close()

        @staticmethod
        def launch_to_csv(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts a .launch to a .csv

                which_file:          The name of the launch file to convert
                                     example: rand.launch

                params["midpoints"]: True if csv should contain midpoint info

                conversion_suffix:   Something to append to the output filename
                                     So if it is "foo", rand.png becomes randfoo.launch.
                """

                filename = which_file.split("/")[-1].split(".")[0]
                car_data_reader = open(which_file)
                car_data = car_data_reader.read()
                car_data_reader.close()
                car_x = car_data.split("<arg name=\"x\" default=\"")[1].split("\"")[0]
                car_y = car_data.split("<arg name=\"y\" default=\"")[1].split("\"")[0]
                car_yaw = car_data.split("<arg name=\"yaw\" default=\"")[1].split("\"")[0]
                midpoints = False
                if "midpoints" in params:
                        midpoints = params["midpoints"]
                Track.runConverter(
                                   filename,
                                   midpoints=midpoints,
                                   car_start_data=("car_start", car_x, car_y, car_yaw, 0.0, 0.0, 0.0),
                                   conversion_suffix=conversion_suffix,
                                   override_name=override_name
                )

        @staticmethod
        def csv_to_png(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts a .csv to a .png

                which_file:        The name of the csv file to convert
                                   example: rand.csv

                params["keep_all_noise"]: True if inactive noise should be kept
                                          Defaults to True
                                          Also applies to lap counters

                conversion_suffix: Something to append to the output filename
                                   So if it is "foo", rand.png becomes randfoo.launch.

                """

                keep_all_noise = params["keep_all_noise"] if "keep_all_noise" in params else True

                filename = which_file.split("/")[-1].split(".")[0]+conversion_suffix
                if override_name is not None:
                    filename = override_name

                # We are going to open up the csv, read through all the lines,
                # and round down the point to an integer after taking into account
                # the scale factor of the csv and potentially scaling accordingly
                # to avoid massive images.
                # (While preserving cone color).

                # First, we read in the csv data into a dataframe
                df = pd.read_csv(which_file)
                blue_cones = df[df['tag'] == "blue"]
                yellow_cones = df[df['tag'] == "yellow"]
                orange_cones = df[df['tag'] == "orange"]
                big_orange_cones = df[df['tag'] == "big_orange"]
                active_noise = df[df['tag'] == "active_noise"]
                inactive_noise = df[df['tag'] == "inactive_noise"]
                car_location = df[df['tag'] == "car_start"]
                lap_counters = df[df['tag'] == "lap_counter"]

                # Here we parse the data and get a full list of all relevant info of the cones
                # and the car (type,x,y,yaw)
                raw_blue = []
                raw_yellow = []
                raw_orange = []
                raw_big_orange = []
                raw_noise = []
                raw_car_location = (0, 0, 0, 0, 0)
                raw_lap_counters = []

                # Note that the indexing may be confusing, pandas has inserted an "index" column,
                # so all indices that you would expect should be shifted upwards by 1
                for bluecone in blue_cones.itertuples():
                        x = (bluecone[2])
                        y = (bluecone[3])
                        raw_blue.append(("blue", 1.0 * x, 1.0 * y, 0))

                for yellowcone in yellow_cones.itertuples():
                        x = (yellowcone[2])
                        y = (yellowcone[3])
                        raw_yellow.append(("yellow", 1.0 * x, 1.0 * y, 0))

                for orangecone in orange_cones.itertuples():
                        x = (orangecone[2])
                        y = (orangecone[3])
                        raw_orange.append(("orange", 1.0 * x, 1.0 * y, 0))

                for big_orangecone in big_orange_cones.itertuples():
                        x = (big_orangecone[2])
                        y = (big_orangecone[3])
                        raw_big_orange.append(("big_orange", 1.0 * x, 1.0 * y, 0))

                for noise in active_noise.itertuples():
                        x = (noise[2])
                        y = (noise[3])
                        raw_noise.append(("noise", 1.0 * x, 1.0 * y, 0))

                if keep_all_noise:
                        for noise in inactive_noise.itertuples():
                                x = (noise[2])
                                y = (noise[3])
                                raw_noise.append(("noise", 1.0 * x, 1.0 * y, 0))

                for c in car_location.itertuples():
                        raw_car_location = ("car", 1.0 * c[2], 1.0 * c[3], c[4])

                for lap_counter in lap_counters.itertuples():
                        x = 1.0*(lap_counter[2])
                        y = 1.0*(lap_counter[3])
                        lap_number = lap_counter[4]
                        raw_lap_counters.append(("lap_counter", x, y, lap_number))

                all_cones = (raw_blue +
                             raw_yellow +
                             raw_orange +
                             raw_big_orange +
                             raw_noise +
                             [raw_car_location] +
                             raw_lap_counters)

                # Here we convert all positions to positive
                min_x = 100000
                min_y = 100000
                max_x = -100000
                max_y = -100000
                for cone in all_cones:
                        if cone[1] < min_x:
                                min_x = cone[1]
                        if cone[2] < min_y:
                                min_y = cone[2]
                        if cone[1] > max_x:
                                max_x = cone[1]
                        if cone[2] > max_y:
                                max_y = cone[2]

                # Here we figure out the track scaling by calculating
                # the average smallest distance between cones
                total_x_distance = 0
                total_y_distance = 0
                for cone1 in all_cones:
                        closest_x = 10000
                        closest_y = 10000
                        for cone2 in all_cones:
                                if cone1 == cone2:
                                    continue
                                dx = abs(cone1[1] - cone2[1])
                                dy = abs(cone1[2] - cone2[2])
                                if dx < closest_x:
                                    closest_x = dx
                                if dy < closest_y:
                                    closest_y = dy
                        total_x_distance += closest_x
                        total_y_distance += closest_y

                # Our scale will strive to preserve distances when possible.
                scale_desired = max(total_x_distance, total_y_distance)/(len(all_cones)-1)
                if scale_desired < 0.0001:
                    scale_desired = 0.0001  # Clamp scale to allowed values
                if scale_desired > 100:
                    scale_desired = 100
                scale_metadata = ConversionTools.deconvert_scale_metadata(scale_desired)

                # Now we store the scaled information inside final_cones/
                final_cones = []
                twidth = int((max_x - min_x + 20) / scale_desired)
                theight = int((max_y - min_y + 20) / scale_desired)
                car_x = int((raw_car_location[1] - min_x + 10) / scale_desired)
                car_y = int((raw_car_location[2] - min_y + 10) / scale_desired)
                for cone in all_cones:
                        new_x = int((cone[1] - min_x + 10) / scale_desired)
                        new_y = int((cone[2] - min_y + 10) / scale_desired)
                        final_cones.append((cone[0], new_x, new_y, cone[3]))

                # Start drawing the track
                im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0))
                draw = ImageDraw.Draw(im)

                # Convert data to image format
                draw.polygon(
                             [(0, 0), (twidth, 0), (twidth, theight), (0, theight), (0, 0)],
                             fill='white'
                )

                # Now we are placing the pixels of our output image
                pixels = im.load()

                def get_cone_color(cone_name, cone_direc):
                        if cone_name == "yellow":
                            return ConversionTools.inner_cone_color
                        elif cone_name == "blue":
                            return ConversionTools.outer_cone_color
                        elif cone_name == "orange":
                            return ConversionTools.orange_cone_color
                        elif cone_name == "big_orange":
                            return ConversionTools.big_orange_cone_color
                        elif cone_name == "noise":
                            return ConversionTools.noise_color
                        elif cone_name == "lap_counter":
                            # Need to add lap number
                            return (
                                ConversionTools.lap_counter_color[0],
                                ConversionTools.lap_counter_color[1],
                                ConversionTools.lap_counter_color[2],
                                255 - int(cone_direc)
                            )
                        elif cone_name == "car":
                            # Calculate the car's yaw's corresponding alpha value
                            yaw_pixel_value = int(raw_car_location[3] / (2 * math.pi) * 254 + 1)
                            if yaw_pixel_value > 255:
                                yaw_pixel_value = 255
                            if yaw_pixel_value < 1:
                                yaw_pixel_value = 1
                            return (
                                ConversionTools.car_color[0],
                                ConversionTools.car_color[1],
                                ConversionTools.car_color[2],
                                yaw_pixel_value
                            )
                        return ConversionTools.inner_cone_color

                # Place the cone pixels
                for cone in final_cones:
                    pixels[cone[1], cone[2]] = get_cone_color(cone[0], cone[3])

                # Add scale metadata:
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  0,
                                                                  0,
                                                                  ConversionTools.TOP_LEFT,
                                                                  im.size
                )
                pixels[loc[0], loc[1]] = scale_metadata[0]
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  1,
                                                                  0,
                                                                  ConversionTools.TOP_LEFT,
                                                                  im.size
                )
                pixels[loc[0], loc[1]] = scale_metadata[1]

                # Add versioning metadata
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  4,
                                                                  4,
                                                                  ConversionTools.BOTTOM_RIGHT,
                                                                  im.size
                )
                pixels[loc[0], loc[1]] = ConversionTools.deconvert_version_metadata(
                                             ConversionTools.TRACKIMG_VERSION_NUM
                )[0]

                # Save the image:
                output_path = os.path.join(
                                           rospkg.RosPack().get_path('eufs_gazebo'),
                                           'randgen_imgs/' + filename + '.png'
                )
                im.save(output_path)
                return im

        @staticmethod
        def csv_to_launch(which_file, params, conversion_suffix="", override_name=None):
                """
                Converts a .csv to a .launch

                which_file:        The name of the csv file to convert
                                   example: rand.csv

                params["keep_all_noise"]: True if inactive noise should be kept
                                          Defaults to True
                                          Also applies to lap counters

                conversion_suffix: Something to append to the output filename
                                   So if it is "foo", rand.png becomes randfoo.launch.


                This contains a lot of duplicated code from csv_to_png and png_to_launch, since
                it's an amalgamation of the two to cut out the semi-deprecated png format.
                Improvements could be made to link these three functions better to reduce loss,
                perhaps by introducing an internal universal data format as an intermediary.
                """

                keep_all_noise = params["keep_all_noise"] if "keep_all_noise" in params else True

                GENERATED_FILENAME = which_file.split("/")[-1].split(".")[0]+conversion_suffix
                if override_name is not None:
                    GENERATED_FILENAME = override_name

                # We are going to open up the csv, read through all the lines,
                # and round down the point to an integer after taking into account
                # the scale factor of the csv and potentially scaling accordingly
                # to avoid massive images.
                # (While preserving cone color).

                # First, we read in the csv data into a dataframe
                df = pd.read_csv(which_file)
                blue_cones = df[df['tag'] == "blue"]
                yellow_cones = df[df['tag'] == "yellow"]
                orange_cones = df[df['tag'] == "orange"]
                big_orange_cones = df[df['tag'] == "big_orange"]
                active_noise = df[df['tag'] == "active_noise"]
                inactive_noise = df[df['tag'] == "inactive_noise"]
                car_location = df[df['tag'] == "car_start"]
                lap_counters = df[df['tag'] == "lap_counter"]

                # Here we parse the data and get a full list of all relevant info of the cones
                # and the car (type,x,y,yaw)
                raw_blue = []
                raw_yellow = []
                raw_orange = []
                raw_big_orange = []
                raw_noise = []
                raw_car_location = (0, 0, 0, 0, 0, 0, 0, 0)
                raw_lap_counters = []

                # Note that the indexing may be confusing, pandas has inserted an "index" column,
                # so all indices that you would expect should be shifted upwards by 1
                for bluecone in blue_cones.itertuples():
                        x = (bluecone[2])
                        y = (bluecone[3])
                        x_cov = bluecone[5]
                        y_cov = bluecone[6]
                        xy_cov = bluecone[7]
                        raw_blue.append(("blue", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

                for yellowcone in yellow_cones.itertuples():
                        x = (yellowcone[2])
                        y = (yellowcone[3])
                        x_cov = bluecone[5]
                        y_cov = bluecone[6]
                        xy_cov = bluecone[7]
                        raw_yellow.append(("yellow", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

                for orangecone in orange_cones.itertuples():
                        x = (orangecone[2])
                        y = (orangecone[3])
                        x_cov = bluecone[5]
                        y_cov = bluecone[6]
                        xy_cov = bluecone[7]
                        raw_orange.append(("orange", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

                for big_orangecone in big_orange_cones.itertuples():
                        x = (big_orangecone[2])
                        y = (big_orangecone[3])
                        x_cov = bluecone[5]
                        y_cov = bluecone[6]
                        xy_cov = bluecone[7]
                        raw_big_orange.append(
                                ("big_orange", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov)
                        )

                for noise in active_noise.itertuples():
                        x = (noise[2])
                        y = (noise[3])
                        x_cov = bluecone[5]
                        y_cov = bluecone[6]
                        xy_cov = bluecone[7]
                        raw_noise.append(("noise", 1.0 * x, 1.0 * y, 0, 0, 0, 0))

                if keep_all_noise:
                        for noise in inactive_noise.itertuples():
                                x = (noise[2])
                                y = (noise[3])
                                raw_noise.append(("noise", 1.0 * x, 1.0 * y, 0, 0, 0, 0))

                for c in car_location.itertuples():
                        raw_car_location = ("car", 1.0 * c[2], 1.0 * c[3], c[4], 0, 0, 0)

                for lap_counter in lap_counters.itertuples():
                        x = 1.0*(lap_counter[2])
                        y = 1.0*(lap_counter[3])
                        lap_number = lap_counter[4]
                        raw_lap_counters.append(("lap_counter", x, y, lap_number, 0, 0, 0))

                all_cones = (raw_blue +
                             raw_yellow +
                             raw_orange +
                             raw_big_orange +
                             raw_noise +
                             raw_lap_counters)

                # Let's start filling in the .launch template:
                launch_template_file = os.path.join(
                                                    rospkg.RosPack().get_path('eufs_launcher'),
                                                    'resource/randgen_launch_template'
                )
                launch_template = open(launch_template_file, "r")

                # .launches need to point to .worlds and model files of the same name,
                # so here we are pasting in copies of the relevant filename.
                launch_merged = "".join(launch_template)
                launch_merged = GENERATED_FILENAME.join(launch_merged.split("%FILLNAME%"))

                # Find the car's position
                # Get x data
                x_pos = raw_car_location[1]
                split_x_data = launch_merged.split("%PLACEX%")
                launch_merged = str(x_pos).join(split_x_data)

                # Get y data
                y_pos = raw_car_location[2]
                split_y_data = launch_merged.split("%PLACEY%")
                launch_merged = str(y_pos).join(split_y_data)

                # Get rotation data
                rot_info = raw_car_location[3]
                split_rot_data = launch_merged.split("%PLACEROTATION%")
                launch_merged = str(rot_info).join(split_rot_data)

                # Here we write out to our launch file.
                launch_out_filename = 'launch/'+GENERATED_FILENAME+".launch"
                launch_out_filepath = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_gazebo'),
                                                   launch_out_filename
                )
                launch_out = open(launch_out_filepath, "w")
                launch_out.write(launch_merged)
                launch_out.close()

                # And now we start the template for the .world:
                world_template_filepath = os.path.join(
                                              rospkg.RosPack().get_path('eufs_launcher'),
                                              'resource/randgen_world_template'
                )
                world_template = open(world_template_filepath, "r")

                # The world file needs to point to the correct model folder,
                # which conveniently has the same name as the world file itself.
                world_merged = "".join(world_template)
                world_merged = GENERATED_FILENAME.join(world_merged.split("%FILLNAME%"))

                # And now we write out.
                world_out_filepath = os.path.join(
                                                  rospkg.RosPack().get_path('eufs_gazebo'),
                                                  'worlds',
                                                  GENERATED_FILENAME+".world"
                )
                world_out = open(world_out_filepath, "w")
                world_out.write(world_merged)
                world_out.close()

                # And now we work on making the model folder:
                # First we create the folder itself
                folder_path = os.path.join(
                                           rospkg.RosPack().get_path('eufs_description'),
                                           'models',
                                           GENERATED_FILENAME
                )

                # If the folder does not exist, which is usual, create it.
                # If the folder does exist, it gets automatically overridden
                # by the rest of this function - this behavior is intended.
                if not os.path.exists(folder_path):
                        os.mkdir(folder_path)

                # Now let's do the .config
                config_template_filepath = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_launcher'),
                                                   'resource/randgen_model_template/model.config'
                )
                config_template = open(config_template_filepath, "r")

                # Let the config file know the name of the track it represents
                config_merged = "".join(config_template)
                config_merged = GENERATED_FILENAME.join(config_merged.split("%FILLNAME%"))

                # Write out the config data
                config_out_filepath = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_description'),
                                                   'models',
                                                   GENERATED_FILENAME,
                                                   "model.config"
                )
                config_out = open(config_out_filepath, "w")
                config_out.write(config_merged)
                config_out.close()

                # Now we create the .sdf
                # This is fairly intensive
                sdf_template_filepath = os.path.join(
                                                     rospkg.RosPack().get_path('eufs_launcher'),
                                                     'resource/randgen_model_template/model.sdf'
                )
                sdf_template = open(sdf_template_filepath, "r")
                sdf_merged = "".join(sdf_template)
                sdf_split_again = sdf_merged.split("$===$")

                # sdf_split_again list contents:
                #        0: Main body of sdf file
                #        1: Outline of noise mesh visual data
                #        2: Outline of noise mesh collision data
                #        3: Noisecube collision data, meant for noise as
                #            a low-complexity collision to prevent falling out the world
                #        4: Outline of noise mesh visual data for innactive noise

                # Here we break up sdf_split_again into its constituent parts
                # At the end of this process we will have:
                #        sdf_main:                  The global template for the sdf
                #        sdf_model:                 The template for cone and noise models
                #        sdf_model_with_collisions: Similar to sdf_model, but with collision data.
                #
                # And the corresponding sdf_ghost_model and sdf_ghost_model_with_collisions
                # which store inactive (hidden) models.
                sdf_main = sdf_split_again[0]
                sdf_split_along_collisions = sdf_split_again[6].split("%FILLCOLLISION%")
                sdf_split_along_ghost_collisions = sdf_split_again[6].split("%FILLCOLLISION%")
                sdf_model = sdf_split_again[3].join(sdf_split_along_collisions)
                sdf_model_with_collisions = sdf_split_again[2].join(sdf_split_along_collisions)
                sdf_ghost_model = sdf_split_again[3].join(sdf_split_along_ghost_collisions)
                sdf_ghost_model_with_collisions = sdf_split_again[2].join(sdf_split_along_ghost_collisions)

                # Let the sdf file know which launch file it represents.
                sdf_main = GENERATED_FILENAME.join(sdf_main.split("%FILLNAME%"))

                # Set up a helper function for calculating model data for cones
                collision_template = sdf_model_with_collisions.split("%MODELNAME%")

                def join_cone_model_data(cone_type):
                        flip_cones = cone_type.split("_")
                        if len(flip_cones) == 1:
                                name = flip_cones[0]
                        else:
                                name = flip_cones[1] + "_" + flip_cones[0]
                        cone_string = "model://models/"+name
                        return cone_string.join(collision_template)

                def setup_covariance(x, y, xy):
                        if not keep_all_noise:
                                # Throw out covariance if directly launching track
                                return ""
                        covariance_template = sdf_split_again[5]
                        output = str(x).join(covariance_template.split("%XCOV%"))
                        output = str(y).join(output.split("%YCOV%"))
                        output = str(xy).join(output.split("%XYCOV%"))
                        return output

                # Calculate model data for cones
                sdf_blue_cone_model = join_cone_model_data("cone_blue")
                sdf_yellow_cone_model = join_cone_model_data("cone_yellow")
                sdf_orange_cone_model = join_cone_model_data("cone_orange")
                sdf_big_orange_cone_model = join_cone_model_data("cone_big")

                # Now let's load in the noise priorities
                noise_priority_file = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_launcher'),
                                                   'resource/noiseFiles.txt'
                )
                noise_files = open(noise_priority_file, "r")
                noise_files = ("".join(noise_files)).split("$===$")[1].strip().split("\n")
                noise_weightings_ = [line.split("|") for line in noise_files]
                noise_weightings = [(float(line[0]), line[1]) for line in noise_weightings_]

                # This function will choose a noise model according to weightings
                # given in the resource/noiseFiles.txt file.
                def get_random_noise_model():
                        randval = uniform(0, 100)
                        for a in noise_weightings:
                                if a[0] > randval:
                                        return a[1]
                        return "model://eufs_description/meshes/NoiseCube.dae"

                # Let's place all the models!
                # We'll keep track of how many we've placed
                # so that we can give each a unique name.
                ConversionTools.link_num = -1

                def put_model_at_position(mod, x, y, modtype, x_cov=0.01, y_cov=0.01, xy_cov=0):
                        """
                        mod: model template to be placed
                        x,y: x and y positions for mod

                        returns model template with x,y, and link_num inserted.
                        """
                        ConversionTools.link_num += 1
                        link_str = str(ConversionTools.link_num)
                        x_str = str((x))
                        y_str = str((y))
                        mod_with_y = y_str.join(mod.split("%PLACEY%"))
                        mod_with_x = x_str.join(mod_with_y.split("%PLACEX%"))
                        mod_with_link = link_str.join(mod_with_x.split("%LINKNUM%"))
                        mod_with_link2 = modtype.join(mod_with_link.split("%LINKTYPE%"))
                        x_cov_str = str(x_cov)
                        y_cov_str = str(y_cov)
                        xy_cov_str = str(xy_cov)
                        mod_with_cov = setup_covariance(x_cov_str, y_cov_str, xy_cov_str).join(
                                mod_with_link2.split("%FILLCOVARIANCE%")
                        )
                        return mod_with_cov

                sdf_allmodels = ""

                def expand_allmodels(allmods, mod, modtype, x, y, x_cov=0.01, y_cov=0.01, xy_cov=0):
                        """
                        Takes in a model and a pixel location,
                        converts the pixel location to a raw location,
                        and places the model inside sdf_allmodels
                        """
                        mod_at_p = put_model_at_position(mod, x, y, modtype, x_cov, y_cov, xy_cov)
                        return allmods + "\n" + mod_at_p

                def get_random_noise_template(mod_with_collisions):
                        """
                        Gets a template for an arbitrary noise model
                        """
                        return get_random_noise_model().join(
                                      mod_with_collisions.split("%MODELNAME%")
                        )

                # Add all models into a template
                color_to_model = {
                        "yellow": sdf_yellow_cone_model,
                        "blue": sdf_blue_cone_model,
                        "orange": sdf_orange_cone_model,
                        "big_orange": sdf_big_orange_cone_model
                }
                for cone in all_cones:
                    name, x, y, direction, x_cov, y_cov, xy_cov = cone
                    if name == "active_noise" or (keep_all_noise and name == "inactive_noise"):
                            if name == "active_noise":
                                    # Noise should be placed
                                    sdf_noisemodel = get_random_noise_template(
                                                       sdf_model_with_collisions
                                    )
                                    sdf_allmodels = expand_allmodels(
                                                           sdf_allmodels,
                                                           sdf_noisemodel,
                                                           name,
                                                           x,
                                                           y
                                    )
                            else:
                                    # Noise exists but not active
                                    sdf_noisemodel = get_random_noise_template(
                                                       sdf_ghost_model_with_collisions
                                    )
                                    sdf_allmodels = expand_allmodels(
                                                           sdf_allmodels,
                                                           sdf_noisemodel,
                                                           name,
                                                           x,
                                                           y
                                    )
                    elif name in color_to_model:
                            # Normal cones.
                            sdf_allmodels = expand_allmodels(
                                                sdf_allmodels,
                                                color_to_model[name],
                                                name + "_cone" if name != "big_orange"
                                                else "big_cone",
                                                x,
                                                y,
                                                x_cov,
                                                y_cov,
                                                xy_cov
                            )
                    elif name == "lap_counter" and keep_all_noise:
                            # Lap counter!
                            sdf_allmodels = expand_allmodels(
                                sdf_allmodels,
                                join_cone_model_data("lap_counter;" + str(int(direction))),
                                "lap_counter;" + str(int(direction)),
                                x,
                                y
                            )

                # Splice the sdf file back together.
                sdf_main = sdf_allmodels.join(sdf_main.split("%FILLDATA%"))

                # Write it out.
                sdf_out_filepath = os.path.join(
                                                rospkg.RosPack().get_path('eufs_description'),
                                                'models',
                                                GENERATED_FILENAME,
                                                "model.sdf"
                )
                sdf_out = open(sdf_out_filepath, "w")
                sdf_out.write(sdf_main)
                sdf_out.close()

        #########################################################
        #                        Copying                        #
        #########################################################

        @staticmethod
        def copy_file(fr, to):
                reader = open(fr, 'r')
                writer = open(to, 'w')
                data = reader.read()
                writer.write(data)
                reader.close()
                writer.close()
