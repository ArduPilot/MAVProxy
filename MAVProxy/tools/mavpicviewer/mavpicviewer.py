#!/usr/bin/env python3

'''
MAV Picture Viewer

Quick and efficient reviewing of images taken from a drone

AP_FLAKE8_CLEAN
'''

import os
from argparse import ArgumentParser
from MAVProxy.modules.lib import multiproc
import mavpicviewer_image
import mavpicviewer_mosaic

prefix_str = "mavpicviewer: "


# main function
if __name__ == "__main__":
    multiproc.freeze_support()
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("filepath", nargs='?', default=".", help="filename or directory holding images")
    args = parser.parse_args()

    # check destination directory exists
    if not os.path.exists(args.filepath):
        exit(prefix_str + "invalid destination directory")

    # create queue for interprocess communication
    mosaic_to_image_comm, image_to_mosaic_comm = multiproc.Pipe()

    # create image viewer
    mavpicviewer_image.mavpicviewer_image(args.filepath, image_to_mosaic_comm)

    # create mosaic
    mavpicviewer_mosaic.mavpicviewer_mosaic(args.filepath, mosaic_to_image_comm)
