#!/usr/bin/env python

import sys
import os
import argparse

def main():
    parser = argparse.ArgumentParser(description='Python wrapper for scrimmage.')
    parser.add_argument('mission_file',
                        type=str,
                        help='SCRIMMAGE mission file.')

    args, unknown = parser.parse_known_args()

    if not os.path.isfile(args.mission_file):
        print('Mission file doesn\'t exist: %s' % args.mission_file)
        return -1

    cmd = "scrimmage " + args.mission_file
    
    return os.system(cmd)

if __name__ == '__main__':
    sys.exit(main())
    
