#!/usr/bin/env python3

import datetime
import os
from ament_index_python.packages import get_package_share_directory
from eufscli import VerbExtension

from eufs_tracks.track_generator import TrackGenerator


class EUFSTracksCreate(VerbExtension):
    '''
    Tool to generate random tracks
    '''

    def configure(self, parser):
        # Main cli arguments
        # TODO: enable cli to produce different formats (e.g launch file)
        parser.add_argument(
            '-o', '--output_file',
            default="track",
            help="csv output file (default='track'). Saves to eufs_tracks shared directory")
        parser.add_argument(
            '-y', '--yes',
            action="store_true",
            help="minimum margin on either side of a track (default: 0)")
        parser.add_argument(
            '-l', '--length',
            type=float,
            help="target track length. Overrides -f and -a flags")
        parser.add_argument(
            '-w', '--track-width',
            type=float,
            help="track width (default: 3)")
        parser.add_argument(
            '--min-cone-spacing',
            type=float,
            help="minimum distance between adjacent cones (default: 3π/16)")
        parser.add_argument(
            '-b', '--cone-bias',
            type=float,
            help="controls the cone spacing on the outside of a turn (default: 0.5)")
        parser.add_argument(
            '-m', '--margin',
            type=float,
            help="minimum margin on either side of a track (default: 0)")

        # Regulation parameters
        regulation_group = parser.add_argument_group("Regulation Parameters")
        regulation_group.add_argument(
            '-r', '--min-corner-radius',
            type=float,
            help="minimum corner radius as measured to the center of the track (default: 3)")
        regulation_group.add_argument(
            '--max-cone-spacing',
            type=float,
            help="maximum distance between adjacent cones (default: 5)")

        # Advanced parameters
        advanced_group = parser.add_argument_group("Advanced Parameters")
        advanced_group.add_argument(
            '--start-straight-length',
            type=float,
            help="the length of the starting straight (default: 6)")
        advanced_group.add_argument(
            '--start-straight-downsample',
            type=int,
            help="reduces the accuracy of the metric used to pick a starting point (default: 2)")
        advanced_group.add_argument(
            '--start-cone-separation',
            type=float,
            help="distance between two big orange cones at the starting line (default: 0.5)")
        advanced_group.add_argument(
            '--rel-accuracy',
            type=float,
            help="maximum relative error in the track length (default: 0.005)")
        advanced_group.add_argument(
            '-f', '--max-freq',
            type=int,
            help="the highest frequency of the waves that make up the path (default: 7)",
            dest="max_frequency")
        advanced_group.add_argument(
            '-a', '--amplitude',
            type=float,
            help="the amplitude of the waves that make up the path (default: 1/3)")
        advanced_group.add_argument(
            '-c', '--check-self-intersects',
            type=float,
            help="guarantees that the generated track has at least margin margin (default: True)")
        advanced_group.add_argument(
            '-s', '--seed',
            type=float,
            help="random number generator seed (default: random)")
        advanced_group.add_argument(
            '-n', '--resolution', '--n-points',
            type=int,
            help="number of points sampled along the curve (default: Non-trivial)")

    def main(self, args):
        # Generate track
        gen = TrackGenerator({k: v for k, v in vars(args).items() if v is not None})
        start_cones, left_cones, right_cones = gen()

        # Save track
        TRACKS_SHARE = get_package_share_directory("eufs_tracks")
        args.output_file = datetime.datetime.today().strftime(args.output_file)
        args.output_file = os.path.join(TRACKS_SHARE, "csv", args.output_file + ".csv")
        try:
            TrackGenerator.write_to_csv(
                args.output_file,
                start_cones,
                left_cones,
                right_cones,
                overwrite=args.yes
            )
        except FileExistsError:
            print(f"The file '{args.output_file}' already exists.")
            overwrite = input("Do you want to replace it? [Y/n]: ")

            if overwrite.lower().startwith('y'):
                TrackGenerator.write_to_csv(
                    args.output_file,
                    start_cones,
                    left_cones,
                    right_cones,
                    overwrite=True
                )
            else:
                print("Abort.")
