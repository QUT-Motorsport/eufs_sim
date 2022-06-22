#!/usr/bin/env python3

from eufscli import CommandExtension, register_entry_point


class EUFSTrackGenerator(CommandExtension):
    """ Tool to generate random tracks """

    def __init__(self):
        self.parser = None

    def configure(self, parser):
        # Save parser to print help text in main()
        self.parser = parser

        # Register verbs
        register_entry_point(parser, self.CLI_NAME, 'verb', 'eufs_tracks.verb')

    def main(self, args):
        # Call the main function of a verb if one is given
        if hasattr(args, 'verb') and args.verb is not None:
            extension = getattr(args, 'verb')
            return extension.main(args=args)
        else:
            self.parser.print_help()
