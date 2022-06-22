#!/usr/bin/env python3

import os
from eufscli import VerbExtension

from ament_index_python.packages import get_package_share_directory

from eufs_tracks.converter_tool import Converter


class EUFSTracksConvert(VerbExtension):
    '''
    Converts tracks between 'launch' and 'csv' formats
    '''

    def configure(self, parser):
        parser.add_argument("track", action="store", help="File path")
        parser.add_argument("fsource", action="store",
                            help="File format of source file ['launch' or 'csv']")
        parser.add_argument("ftarget", action="store",
                            help="File format of target file ['launch' or 'csv']")
        parser.add_argument("-n", "--name", action="store", dest="name",
                            default="", help="Name of target")

    def main(self, args):
        assert args.fsource in ["launch", "csv"], "fsource must be either 'launch' or 'csv'"
        assert args.ftarget in ["launch", "csv"], "ftarget must be either 'launch' or 'csv'"

        TRACKS_SHARE = get_package_share_directory("eufs_tracks")
        # Check if file is in current directory
        if not os.path.exists(args.track):
            if args.fsource == "csv":
                args.track = os.path.join(TRACKS_SHARE, 'csv', args.track + ".csv")
            else:
                args.track = os.path.join(TRACKS_SHARE, 'launch', args.track + ".launch")

        # Add name override if provided
        params = {'override_name': args.name} if args.name != "" else {}

        # Convert track
        Converter.convert(args.fsource, args.ftarget, args.track, params)
