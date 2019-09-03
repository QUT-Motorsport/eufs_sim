from PIL import Image
from PIL import ImageDraw
import math
from LauncherUtilities import calculate_tangent_angle
from random import randrange, uniform
import os
import rospkg
import rospy
import sys
sys.path.insert(1, os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),'tracks'))
from track_gen import Track
from TrackGenerator import compactify_points
import pandas as pd

# Here are all the track formats we care about:
# .launch (well, we actually want the model data, not the .launch, but we'll treat it as wanting the .launch)
#                (since the end user shouldn't have to care about the distinction)
# .png
# .csv
# raw_data ("xys",this will be hidden from the user as it is only used to convert into .pngs)
class ConversionTools:
        def __init__():
                pass

        # Define colors for track gen and image reading
        noise_color = (0,255,255,255)               #cyan, the turquoise turqouise wishes it were
        background_color = (255,255,255,255)        #white
        inner_cone_color  = (255,0,255,255)         #magenta, for yellow cones (oops...)
        outer_cone_color = (0,0,255,255)            #blue, for blue cones
        car_color = (0,255,0,255)                   #green
        track_center_color = (0,0,0,255)            #black
        track_inner_color = (255,0,0,255)           #red
        track_outer_color = (255,255,0,255)         #yellow
        orange_cone_color = (255,165,0,255)         #orange, for orange cones
        big_orange_cone_color = (127,80,0,255)      #dark orange, for big orange cones        

        # Other various retained parameters
        link_num = -1
        TRACKIMG_VERSION_NUM = 1

        # Metadata corner variables
        TOP_LEFT     = "Top Left"
        BOTTOM_LEFT  = "Bottom Left"
        TOP_RIGHT    = "Top Right"
        BOTTOM_RIGHT = "Bottom Right"

        #########################################################
        #              Handle Track Image Metadata              #
        #########################################################

        @staticmethod
        def get_metadata_pixel(x,y,corner,pixels,size):
                """
                Returns the metadata pixel (x,y,corner)'s value.
                
                x: x position within the corner
                y: y position within the corner
                corner: which corner of the image we are in
                pixels: pixels of the image
                size: size of the image
                """

                x_out,y_out = ConversionTools.get_metadata_pixel_location(x,y,corner,size)
                return pixels[x_out,y_out]

        @staticmethod
        def get_metadata_pixel_location(x,y,corner,size):
                """Returns the metadata pixel (x,y,corner)'s location on the actual image."""
                width = size[0]
                height = size[1]
                if corner   == ConversionTools.TOP_LEFT:
                        return (x,y)
                elif corner == ConversionTools.BOTTOM_LEFT:
                        return (x,height-6+y)
                elif corner == ConversionTools.TOP_RIGHT:
                        return (width-6+x,y)
                elif corner == ConversionTools.BOTTOM_RIGHT:
                        return (width-6+x,height-6+y)
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
                (r,g,b,a) = pixel_value
                return a-1 + (b-1)*254 + (g-1)*254**2 + (r-1)*254**3

        @staticmethod
        def unget_raw_metadata(metadata):
                """Undoes the process of get_raw_metadata()"""
                a = metadata % 254
                metadata = (metadata - a) // 254
                b = metadata % 254
                metadata = (metadata - b) // 254
                g = metadata % 254
                r = (metadata - g) // 254

                return (r+1,g+1,b+1,a+1)

        @staticmethod
        def convert_scale_metadata(pixel_values):
                """
                This function converts the data obtained from 
                scale metadata pixels into actual scale information
                Output range is from 0.0001 to 100.
                """                

                primary_pixel = pixel_values[0]
                secondary_pixel = pixel_values[1]

                #Check for default pixel values
                if (primary_pixel == (255,255,255,255) and
                   secondary_pixel == (255,255,255,255)):
                        return 1

                metadata = ConversionTools.get_raw_metadata(primary_pixel)

                # Want to linearly transform the metadata, a range from 0 to 254**4-1, to the range 0.0001 to 100
                return metadata/(254**4-1.0) * (100-0.0001) + 0.0001


        @staticmethod
        def deconvert_scale_metadata(data):
                """
                This function converts a raw_ scale value into a 
                list of metadata pixels needed to replicate it.
                First in list is the primary metadata pixel, second in list is the secondary
                """

                metadata = int((data-0.0001)/(100-0.0001) * (254**4-1))
                primary_pixel = ConversionTools.unget_raw_metadata(metadata)
                secondary_pixel = (255,255,255,255)
                return [primary_pixel,secondary_pixel]

        @staticmethod
        def convert_version_metadata(pixel_values):
                """
                This function converts the data obtained from 
                scale metadata pixels into actual scale information
                Output range is from 0 to 254**4-1
                """
                primary_pixel = pixel_values[0]
                if primary_pixel == (255,255,255,255): return 0
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
        def convert(cfrom,cto,which_file,params=[],conversion_suffix=""):
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

                if cfrom=="xys" and cto=="png":
                        return ConversionTools.xys_to_png(which_file,params,conversion_suffix)
                if cfrom=="png" and cto=="launch":
                        return ConversionTools.png_to_launch(which_file,params,conversion_suffix)
                if cfrom=="png" and cto=="csv":
                        ConversionTools.png_to_launch(which_file,params,conversion_suffix)
                        new_file_array = which_file.split("/")
                        new_file_array[-2] = "launch"
                        which_file = "/".join(new_file_array)
                        return ConversionTools.launch_to_csv(
                                   which_file[:-4]+conversion_suffix+".launch",
                                   params,
                                   conversion_suffix=""
                        )
                if cfrom=="launch" and cto=="csv":
                        return ConversionTools.launch_to_csv(which_file,params,conversion_suffix)
                if cfrom=="launch" and cto=="png":
                        ConversionTools.launch_to_csv(which_file,params,conversion_suffix)
                        new_file_array = which_file.split("/")
                        new_file_array[-2] = "tracks"
                        which_file = "/".join(new_file_array)
                        return ConversionTools.csv_to_png(
                                   which_file[:-7]+conversion_suffix+".csv",
                                   params,
                                   conversion_suffix=""
                        )
                if cfrom=="csv" and cto == "launch":
                        ConversionTools.csv_to_png(which_file,params,conversion_suffix)
                        new_file_array = which_file.split("/")
                        new_file_array[-2] = "randgen_imgs"
                        which_file = "/".join(new_file_array)
                        return ConversionTools.png_to_launch(
                                   which_file[:-4]+conversion_suffix+".png",
                                   params,
                                   conversion_suffix=""
                        )
                if cfrom=="csv" and cto == "png":
                        return ConversionTools.csv_to_png(which_file,params,conversion_suffix)
                if cto == "ALL" or cto == "all":
                        #If something tries to convert to itself it just gets ignored
                        ConversionTools.convert(cfrom,"launch",which_file,params,conversion_suffix)
                        ConversionTools.convert(cfrom,"csv",which_file,params,conversion_suffix)
                        return ConversionTools.convert(
                                   cfrom,
                                   "png",
                                   which_file,
                                   params,
                                   conversion_suffix
                        )
                return None
                        

        @staticmethod
        def xys_to_png(which_file,params,conversion_suffix=""):
                """
                Converts xys format to png.

                which_file:        Output filename
                params[0]:         Track width (cone distance)
                params[1]:         Tuple of: list of points, image width, image height
                conversion_suffix: Additional suffix to append to the output filename.
                """

                GENERATED_FILENAME = which_file + conversion_suffix

                # Unpack
                (xys,twidth,theight) = params[1]
                cone_normal_distance_parameter = params[0]

                # Create image to hold data
                im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0)) 
                draw = ImageDraw.Draw(im)

                # Draw backround:
                draw.polygon([(0,0),(twidth,0),(twidth,theight),(0,theight),(0,0)],fill='white')

                # Draw thick track with colored layers
                draw.line(xys,fill=ConversionTools.track_outer_color,width=5)
                draw.line(xys,fill=ConversionTools.track_inner_color,width=3)
                draw.line(xys,fill=ConversionTools.track_center_color)

                pixels = im.load()

                # Get a list of integerized points to work well with 
                # integer-valued pixel locations.
                compact_xys = compactify_points([(int(xy[0]),int(xy[1])) for xy in xys])

                # We want to calculate direction of car position
                sx = xys[0][0]
                sy = xys[0][1]
                ex = xys[1][0]
                ey = xys[1][1]

                # So we calculate the angle that the car is facing (the yaw)
                angle = math.atan2(ey-sy,ex-sx)
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
                pixel_value = int(angle / ( 2 * math.pi ) * 254 + 1)
                if pixel_value > 255: pixel_value = 255
                if pixel_value <   1: pixel_value =   1

                # Draw car
                color_for_car = ConversionTools.car_color[:3]+(pixel_value,)
                draw.line([xys[0],xys[0]],fill=color_for_car)

                def is_track(c):
                        return c == ConversionTools.track_outer_color or \
                               c == ConversionTools.track_inner_color or \
                               c == ConversionTools.track_center_color

                # Now we want to make all pixels bordering the track become magenta (255,0,255) -
                # this will be our 'cone' color
                # To find pixel boardering track, simply find white pixel adjacent to 
                # a non-white non-magenta pixle
                # We will also want to make it such that cones are about 4-6 
                # away from eachother euclideanly        


                all_points_north = []
                all_points_south = []
                prev_points_north = [(-10000,-10000)]
                prev_points_south = [(-10000,-10000)]
                print("Placing Cones...")
                for i in range(len(xys)):
                        #Skip first part [as hard to calculate tangent]
                        if i == 0: continue
                
                        # The idea here is to place cones along normals to the 
                        # tangent at any given point, 
                        # while making sure they aren't too close.

                        # Hardcoded parameters (not available in launcher because the set of 
                        # well-behaved answers is far from dense in the problemspace):

                        # How close can cones be from those on opposite side.
                        cone_cross_closeness_parameter = cone_normal_distance_parameter*3/4-1

                        # Larger numbers makes us check more parts of the track for conflicts.
                        cone_check_amount = 30

                        # How close can cones be from those on the same side.
                        cone_adjacent_closeness_parameter = 6

                        # Here we calculate the normal vectors along the track
                        # As we want cones to be placed along the normal.
                        cur_point = xys[i]
                        cur_tangent_angle = calculate_tangent_angle(xys[:(i+1)])
                        cur_tangent_normal = (
                            math.ceil(
                                cone_normal_distance_parameter * math.sin(cur_tangent_angle)
                            ),
                            math.ceil(
                                -cone_normal_distance_parameter * math.cos(cur_tangent_angle)
                            )
                        )

                        # This is where the cones will be placed, provided they pass the
                        # distance checks later.
                        north_point = (int(cur_point[0] + cur_tangent_normal[0]),
                                       int(cur_point[1] + cur_tangent_normal[1]))
                        south_point = (int(cur_point[0] - cur_tangent_normal[0]), 
                                       int(cur_point[1] - cur_tangent_normal[1]))

                        # Calculates shortest distance to cone on same side of track
                        difference_from_prev_north = min([
                                        (north_point[0]-prev_point_north[0])**2
                                       +(north_point[1]-prev_point_north[1])**2
                                for prev_point_north in prev_points_north
                        ])
                        difference_from_prev_south = min([
                                        (south_point[0]-prev_point_south[0])**2
                                       +(south_point[1]-prev_point_south[1])**2
                                for prev_point_south in prev_points_south
                        ])

                        # Calculates shortest distance to cone on different side of track
                        cross_distance_ns = min([
                                        (north_point[0]-prev_point_south[0])**2
                                       +(north_point[1]-prev_point_south[1])**2
                                for prev_point_south in prev_points_south
                        ])
                        cross_distance_sn = min([
                                        (south_point[0]-prev_point_north[0])**2
                                       +(south_point[1]-prev_point_north[1])**2
                                for prev_point_north in prev_points_north
                        ])

                        # Here we ensure cones don't get too close to the track
                        rel_xys = xys[ 
                            max([0,i-cone_check_amount]): 
                            min([len(xys),i+cone_check_amount])  
                        ]
                        distance_pn = min([
                                            (north_point[0]-xy[0])**2
                                           +(north_point[1]-xy[1])**2 
                                                for xy in rel_xys
                        ])
                        distance_ps = min([
                                            (south_point[0]-xy[0])**2
                                           +(south_point[1]-xy[1])**2 
                                                for xy in rel_xys
                        ])

                        # And we consider all these factors to see if the cones are viable
                        north_viable = (
                              difference_from_prev_north > cone_adjacent_closeness_parameter**2
                          and cross_distance_ns > cone_cross_closeness_parameter**2
                          and distance_pn > cone_cross_closeness_parameter**2
                        )
                        south_viable = (
                              difference_from_prev_south > cone_adjacent_closeness_parameter**2
                          and cross_distance_sn > cone_cross_closeness_parameter**2
                          and distance_ps > cone_cross_closeness_parameter**2
                        )

                        # And when they are viable, draw them!
                        if (
                            not is_track(pixels[int(north_point[0]),int(north_point[1])]) 
                            and north_viable
                        ):
                                px_x = int(north_point[0])
                                px_y = int(north_point[1])
                                pixels[px_x,px_y]=ConversionTools.inner_cone_color
                                all_points_north.append(north_point)
                        if (
                            not is_track(pixels[int(south_point[0]),int(south_point[1])]) 
                            and south_viable
                        ):
                                px_x = int(south_point[0])
                                px_y = int(south_point[1])
                                pixels[px_x,px_y]=ConversionTools.inner_cone_color
                                all_points_south.append(south_point)

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
                                prev_points_north = (all_points_north[-cone_check_amount:]
                                                  + [all_points_north[0]])
                        elif len(all_points_north) > 0:
                                prev_points_north = all_points_north

                        if len(all_points_south) > cone_check_amount:
                                prev_points_south = (all_points_south[-cone_check_amount:]
                                                  + [all_points_south[0]])
                        elif len(all_points_south) > 0:
                                prev_points_south = all_points_south

                
                
                # We want to make the track have differing cone colors - 
                # yellow on inside, blue on outside
                # All cones are currently yellow.  
                # We'll do a "fill-search" for any cones reachable
                # from (0,0) and call those the 'outside' 
                # as (0,0) is always blank since the image is padded)
                # So we will return a list of all points that the search can reach.
                # Precisely, this is a list of all points adjacent to curpix that
                # have not already been explored.
                def get_allowed_adjacents(explolist,curpix):
                        (i,j) = curpix
                        allowed = []
                        if i > 0:
                                if not((i-1, j) in explolist):
                                        allowed.append((i-1, j))
                        if j > 0:
                                if not((i,j-1) in explolist):
                                        allowed.append((i, j-1))
                        if i < twidth-1:
                                if not((i+1,j) in explolist):
                                        allowed.append((i+1, j))
                        if j < theight-1:
                                if not((i,j+1) in explolist):
                                        allowed.append((i, j+1))
                        return allowed
                print("Coloring cones...")

                # This is the bread-and-butter of the search algorithm.
                # All it is doing is checking if the points in frontier are
                # a cone, and if they are then swap it to the correct cone color.
                # (all cones are inner_cone_color by default, but those on the outside
                # of the track must be swapped to outer_cone_color).
                # When we search a pixel, we remove it from the frontier and
                # add its adjacents to the frontier (provided they are not already explored).
                # We only do this if the pixel is a background or cone pixel, otherwise
                # we do not add its adjacents to the frontier.
                # We don't want the search to be able to pass through track pixels, as then
                # we would end up on the "inside" of the track, and would wrongfully color
                # inner_cones as outer_cones.
                explored_list = set([])
                frontier = set([(0,0)])
                while len(frontier)>0:
                        new_frontier = set([])
                        for f in frontier:
                                (i,j) = f
                                pix = pixels[i,j]
                                if pix == ConversionTools.inner_cone_color:
                                        pixels[i,j] = ConversionTools.outer_cone_color
                                        new_frontier.update(
                                                get_allowed_adjacents(explored_list,(i,j))
                                        )
                                elif pix == ConversionTools.background_color:
                                        new_frontier.update(
                                                get_allowed_adjacents(explored_list,(i,j))
                                        )
                                explored_list.add(f)
                        frontier = new_frontier

                # Add orange start cones
                (i,j) = all_points_north[0]
                pixels[int(i),int(j)] = ConversionTools.orange_cone_color
                (i,j) = all_points_south[0]
                pixels[int(i),int(j)] = ConversionTools.orange_cone_color

                # Finally, we just need to place noise.  
                # At maximal noise, the track should be about 1% covered.

                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                # Don't add noise in margin
                                if i<5 or j<5 or i>=im.size[0]-5 or j>=im.size[0]-5: 
                                        continue
                                if pixels[i,j] == ConversionTools.background_color:
                                        # Only 1% should be covered with noise
                                        if uniform(0,100) < 1:
                                                pixels[i,j] = ConversionTools.noise_color

                # Add margins (as specified by the file format)
                margin = 5
                im2 = Image.new('RGBA', (twidth+2*margin, theight+2*margin), (255, 255, 255, 255)) 
                pixels2 = im2.load()
                for x in range(im.size[0]):
                        for y in range(im.size[1]):
                                pixels2[x+margin,y+margin] = pixels[x,y]

                # And tag it with the version number
                loc = ConversionTools.get_metadata_pixel_location(
                                            4,
                                            4,
                                            ConversionTools.BOTTOM_RIGHT,
                                            im2.size
                )
                pixels2[loc[0],loc[1]] = ConversionTools.deconvert_version_metadata(
                                             ConversionTools.TRACKIMG_VERSION_NUM
                )[0]


                im2.save(
                        os.path.join(rospkg.RosPack().get_path('eufs_gazebo'),
                        'randgen_imgs/'+GENERATED_FILENAME+'.png')
                )
                return im

        @staticmethod
        def png_to_launch(which_file,params=[0],conversion_suffix=""):
                """
                Converts a .png to a .launch with .world, model.sdf, model.config files.

                which_file:        The name of the png file to convert
                                   example: rand.png

                params:            A list of optional parameters.  
                                   The first component should be the noise level 
                                   (on range [0,1])

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
                im = Image.open(which_file)
                noise_level = params[0]
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
                version_number = ConversionTools.convert_version_metadata([pixels[loc[0],loc[1]]])

                # Let's start filling in the .launch template:
                launch_template_file = os.path.join(
                                                    rospkg.RosPack().get_path('eufs_launcher'),
                                                    'resource/randgen_launch_template'
                )
                launch_template = open(launch_template_file,"r")

                # .launches need to point to .worlds and model files of the same name,
                # so here we are pasting in copies of the relevant filename.
                launch_merged = "".join(launch_template)
                launch_merged = GENERATED_FILENAME.join(launch_merged.split("%FILLNAME%"))

                # These are helper functions to convert pixel data, such as coordinates,
                # into raw numerical data for the .launch file.
                def x_coord_transform(x):
                        return x-50
                def y_coord_transform(y):
                        return y-50
                def is_car_color(x):
                        return x[:-1]==ConversionTools.car_color[:-1]
                def rotation_transform(x):
                        return 2*math.pi*((x-1)/254.0)

                # Find the car's position
                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                p = pixels[i,j]
                                if is_car_color(p):
                                        # Get x data
                                        x_pos = x_coord_transform(i*scale_data)
                                        split_x_data = launch_merged.split("%PLACEX%")
                                        launch_merged = str(x_pos).join(split_x_data)
 
                                        # Get y data
                                        y_pos = y_coord_transform(j*scale_data)
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
                world_template = open(world_template_filepath,"r")

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
                world_out = open(world_out_filepath,"w")
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
                config_template = open(config_template_filepath,"r")

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
                config_out = open(config_out_filepath,"w")
                config_out.write(config_merged)
                config_out.close()

                # Now we create the .sdf
                # This is fairly intensive
                sdf_template_filepath = os.path.join(
                                                     rospkg.RosPack().get_path('eufs_launcher'),
                                                     'resource/randgen_model_template/model.sdf'
                )
                sdf_template = open(sdf_template_filepath,"r")
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
                sdf_main                         = sdf_split_again[0]
                sdf_split_along_collisions       = sdf_split_again[1].split("%FILLCOLLISION%")
                sdf_split_along_ghost_collisions = sdf_split_again[4].split("%FILLCOLLISION%")
                sdf_model                        = sdf_split_again[3].join(
                                                       sdf_split_along_collisions
                )
                sdf_model_with_collisions        = sdf_split_again[2].join(
                                                       sdf_split_along_collisions
                )
                sdf_ghost_model                  = sdf_split_again[3].join(
                                                       sdf_split_along_ghost_collisions
                )
                sdf_ghost_model_with_collisions  = sdf_split_again[2].join(
                                                       sdf_split_along_ghost_collisions
                )

                # Let the sdf file know which launch file it represents.
                sdf_main = GENERATED_FILENAME.join(sdf_main.split("%FILLNAME%"))

                # Set up a helper function for calculating model data for cones
                collision_template = sdf_model_with_collisions.split("%MODELNAME%")
                def join_cone_model_data(cone_type):
                        cone_string = "model://eufs_description/meshes/"+cone_type+".dae"
                        return cone_string.join(collision_template)

                # Calculate model data for cones
                sdf_blue_cone_model = join_cone_model_data("cone_blue")
                sdf_yellow_cone_model =  join_cone_model_data("cone_yellow")
                sdf_orange_cone_model =  join_cone_model_data("cone")
                sdf_big_orange_cone_model =  join_cone_model_data("cone_big")

                # Now let's load in the noise priorities
                noise_priority_file = os.path.join(
                                                   rospkg.RosPack().get_path('eufs_launcher'),
                                                   'resource/noiseFiles.txt'
                )
                noise_files = open(noise_priority_file,"r")
                noise_files = ("".join(noise_files)).split("$===$")[1].strip().split("\n")
                noise_weightings_ = [line.split("|") for line in noise_files]
                noise_weightings  = [(float(line[0]),line[1]) for line in noise_weightings_]

                # This function will choose a noise model according to weightings
                # given in the resource/noiseFiles.txt file.
                def get_random_noise_model():
                        randval = uniform(0,100)
                        for a in noise_weightings:
                                if a[0]>randval:
                                        return a[1]
                        return "model://eufs_description/meshes/NoiseCube.dae"

                # Let's place all the models!
                # We'll keep track of how many we've placed 
                # so that we can give each a unique name.
                ConversionTools.link_num = -1
                def put_model_at_position(mod,x,y):
                        """
                        mod: model template to be placed
                        x,y: x and y positions for mod
                        
                        returns model template with x,y, and link_num inserted.
                        """
                        ConversionTools.link_num+=1
                        link_str = str(ConversionTools.link_num)
                        x_str = str(x_coord_transform(x))
                        y_str = str(y_coord_transform(y))
                        mod_with_y = y_str.join(mod.split("%PLACEY%"))
                        mod_with_x = x_str.join(mod_with_y.split("%PLACEX%"))
                        mod_with_link = link_str.join(mod_with_x.split("%link_num%"))
                        return mod_with_link

                sdf_allmodels = ""
                def expand_allmodels(allmods,mod,x,y):
                        """
                        Takes in a model and a pixel location,
                        converts the pixel location to a raw location,
                        and places the model inside sdf_allmodels
                        """
                        mod_at_p =  put_model_at_position(mod,x*scale_data,y*scale_data)
                        return allmods + "\n" + mod_at_p

                def get_random_noise_template(mod_with_collisions):
                        """
                        Gets a template for an arbitrary noise model
                        """
                        return get_random_noise_model().join(
                                      mod_with_collisions.split("%MODELNAME%")
                        )


                # Add all models into a template
                for i in range(im.size[0]):
                        for j in range(im.size[1]):
                                p = pixels[i,j]
                                if p == ConversionTools.inner_cone_color:
                                        sdf_allmodels = expand_allmodels(
                                                            sdf_allmodels,
                                                            sdf_yellow_cone_model,
                                                            i,
                                                            j
                                        )
                                elif p == ConversionTools.outer_cone_color:
                                        sdf_allmodels = expand_allmodels(
                                                            sdf_allmodels,
                                                            sdf_blue_cone_model,
                                                            i,
                                                            j
                                        )
                                elif p == ConversionTools.orange_cone_color:
                                        sdf_allmodels = expand_allmodels(
                                                            sdf_allmodels,
                                                            sdf_orange_cone_model,
                                                            i,
                                                            j
                                        )
                                elif p == ConversionTools.big_orange_cone_color:
                                        sdf_allmodels = expand_allmodels(
                                                            sdf_allmodels,
                                                            sdf_big_orange_cone_model,
                                                            i,
                                                            j
                                        )
                                elif p == ConversionTools.noise_color:
                                        if uniform(0,1)<noise_level:
                                                # Noise should be placed
                                                sdf_noisemodel = get_random_noise_template(
                                                                   sdf_model_with_collisions
                                                )
                                                sdf_allmodels = expand_allmodels(
                                                                       sdf_allmodels,
                                                                       sdf_noisemodel,
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
                sdf_out = open(sdf_out_filepath,"w")
                sdf_out.write(sdf_main)
                sdf_out.close()
                

        @staticmethod
        def launch_to_csv(which_file,params=[0],conversion_suffix=""):
                filename = which_file.split("/")[-1].split(".")[0]
                car_data_reader = open(which_file)
                car_data = car_data_reader.read()
                car_data_reader.close()
                car_x   = car_data.split("<arg name=\"x\" default=\"")[1].split("\"")[0]
                car_y   = car_data.split("<arg name=\"y\" default=\"")[1].split("\"")[0]
                car_yaw = car_data.split("<arg name=\"yaw\" default=\"")[1].split("\"")[0]
                midpoints=False
                if len(params) >= 2:
                        midpoints = params[1]
                Track.runConverter(filename,midpoints=midpoints,car_start_data=("car_start",car_x,car_y,car_yaw),conversion_suffix = conversion_suffix)


        @staticmethod
        def csv_to_png(which_file,params,conversion_suffix=""):
                """
                Converts a .csv to a .png

                which_file:        The name of the csv file to convert
                                   example: rand.csv

                params:            A list of optional parameters.  
                                   None are used at this time, but it is listed in the
                                   function signature to have a consistent signature for all
                                   the methods of the form a_to_b().

                conversion_suffix: Something to append to the output filename
                                   So if it is "foo", rand.png becomes randfoo.launch.

                """

                filename = which_file.split("/")[-1].split(".")[0]+conversion_suffix

                # We are going to open up the csv, read through all the lines, 
                # and round down the point to an integer after taking into account
                # the scale factor of the csv and potentially scaling accordingly
                # to avoid massive images.
                # (While preserving cone color).

                # First, we read in the csv data into a dataframe
                df = pd.read_csv(which_file)
                blue_cones = df[df['tag']=="blue"]
                yellow_cones = df[df['tag']=="yellow"]
                orange_cones = df[df['tag']=="orange"]
                big_orange_cones = df[df['tag']=="big_orange"]
                active_noise = df[df['tag']=="active_noise"]
                inactive_noise = df[df['tag']=="inactive_noise"]
                car_location = df[df['tag']=="car_start"]

                # Here we parse the data and get a full list of all relevant info of the cones
                # and the car (type,x,y,yaw)
                raw_blue = []
                raw_yellow = []
                raw_orange = []
                raw_big_orange = []
                raw_noise = []
                raw_car_location = (0, 0, 0, 0)
                for bluecone in blue_cones.itertuples():
                        x = (bluecone[2])
                        y = (bluecone[3])
                        raw_blue.append(("blue", x, y, 0))

                for yellowcone in yellow_cones.itertuples():
                        x = (yellowcone[2])
                        y = (yellowcone[3])
                        raw_yellow.append(("yellow", x, y, 0))

                for orangecone in orange_cones.itertuples():
                        x = (orangecone[2])
                        y = (orangecone[3])
                        raw_orange.append(("orange", x, y, 0))

                for big_orangecone in big_orange_cones.itertuples():
                        x = (big_orangecone[2])
                        y = (big_orangecone[3])
                        raw_big_orange.append(("big_orange", x, y, 0))

                for noise in active_noise.itertuples():
                        x = (noise[2])
                        y = (noise[3])
                        raw_noise.append(("noise", x, y, 0))

                for noise in inactive_noise.itertuples():
                        x = (noise[2])
                        y = (noise[3])
                        raw_noise.append(("noise", x, y, 0))

                for c in car_location.itertuples():
                        raw_car_location = ("car", c[2], c[3], c[4])

                all_cones = (raw_blue 
                           + raw_yellow 
                           + raw_orange 
                           + raw_big_orange 
                           + raw_noise 
                           + [raw_car_location])

                # Here we convert all positions to positive
                min_x = 100000
                min_y = 100000
                max_x = -100000
                max_y = -100000
                for cone in all_cones:
                        if cone[1]<min_x:
                                min_x = cone[1]
                        if cone[2]<min_y:
                                min_y = cone[2]
                        if cone[1]>max_x:
                                max_x = cone[1]
                        if cone[2]>max_y:
                                max_y = cone[2]

                # Here we figure out the track scaling by calculating 
                # the average smallest distance between cones
                total_x_distance = 0
                total_y_distance = 0
                for cone1 in all_cones:
                        closest_x = 10000
                        closest_y = 10000
                        for cone2 in all_cones:
                                if cone1 == cone2: continue
                                dx = abs(cone1[1]-cone2[1])
                                dy = abs(cone1[2]-cone2[2])
                                if dx < closest_x: closest_x = dx
                                if dy < closest_y: closest_y = dy
                        total_x_distance+=closest_x
                        total_y_distance+=closest_y
                
                # Our scale will strive to preserve distances when possible.
                scale_desired = max(total_x_distance,total_y_distance)/(len(all_cones)-1)
                if scale_desired < 0.0001: scale_desired = 0.0001#Clamp scale to allowed values
                if scale_desired > 100:    scale_desired = 100
                scale_metadata = ConversionTools.deconvert_scale_metadata(scale_desired)
                
                # Now we store the scaled information inside final_cones/
                final_cones = []
                twidth =  int((max_x - min_x + 20) / scale_desired)
                theight = int((max_y - min_y + 20) / scale_desired)
                car_x = int((raw_car_location[1] - min_x + 10) / scale_desired)
                car_y = int((raw_car_location[2] - min_y + 10) / scale_desired)
                for cone in all_cones:
                        new_x = int((cone[1] - min_x + 10) / scale_desired)
                        new_y = int((cone[2] - min_y + 10) / scale_desired)
                        final_cones.append((cone[0], new_x, new_y))

                # Start drawing the track
                im = Image.new('RGBA', (twidth, theight), (0, 0, 0, 0)) 
                draw = ImageDraw.Draw(im)

                # Convert data to image format
                draw.polygon(
                             [(0,0),(twidth,0),(twidth,theight),(0,theight),(0,0)],
                             fill='white'
                )

                # Now we are placing the pixels of our output image
                pixels = im.load()
                def get_cone_color(cone_name):
                        if   cone_name == "yellow":     
                                return ConversionTools.inner_cone_color
                        elif cone_name == "blue":       
                                return ConversionTools.outer_cone_color
                        elif cone_name == "orange":     
                                return ConversionTools.orange_cone_color
                        elif cone_name == "big_orange": 
                                return ConversionTools.big_orange_cone_color
                        elif cone_name == "noise":      
                                return ConversionTools.noise_color
                        return ConversionTools.inner_cone_color

                # Place the cone pixels
                for cone in final_cones:
                        pixels[cone[1],cone[2]] = get_cone_color(cone[0])

                # Calculate the car's yaw's corresponding alpha value
                pixel_value = int(raw_car_location[3]/(2*math.pi)*254+1) 
                if pixel_value > 255: pixel_value = 255
                if pixel_value <   1: pixel_value =   1

                # Finally, add the car
                pixels[car_x,car_y] = (
                                       ConversionTools.car_color[0],
                                       ConversionTools.car_color[1],
                                       ConversionTools.car_color[2],
                                       pixel_value
                )
                
                # Add scale metadata:
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  0,
                                                                  0,
                                                                  ConversionTools.TOP_LEFT,
                                                                  im.size
                )
                pixels[loc[0],loc[1]] = scale_metadata[0]
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  1,
                                                                  0,
                                                                  ConversionTools.TOP_LEFT,
                                                                  im.size
                )
                pixels[loc[0],loc[1]] = scale_metadata[1]

                # Add versioning metadata
                loc = ConversionTools.get_metadata_pixel_location(
                                                                  4,
                                                                  4,
                                                                  ConversionTools.BOTTOM_RIGHT,
                                                                  im.size
                )
                pixels[loc[0],loc[1]] = ConversionTools.deconvert_version_metadata(
                                             ConversionTools.TRACKIMG_VERSION_NUM
                )[0]

                # Save the image:
                output_path = os.path.join(
                                           rospkg.RosPack().get_path('eufs_gazebo'),
                                           'randgen_imgs/'+filename+'.png'
                )
                im.save(output_path)
                return im
                


        #########################################################
        #                        Copying                        #
        #########################################################
        
        @staticmethod
        def copy_file(fr,to):
                reader = open(fr,'r')
                writer = open(to,'w')
                data = reader.read()
                writer.write(data)
                reader.close()
                writer.close()
                
