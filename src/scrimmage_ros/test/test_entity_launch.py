#!/usr/bin/env python

import os
import sys
import argparse

from scrimmage_ros.EntityLaunch import EntityLaunch
from scrimmage_ros.MultiProcessLogger import Terminal
import scrimmage_ros.utils as sru

def main():

    parser = argparse.ArgumentParser(
        description='Runs the autonomy in HIL or SIL.')
    parser.add_argument('-s', '--sim_mission_yaml_file',
                        default='$(find scrimmage_ros)/src/scrimmage_ros/test/templates/nav_2d_mission.yaml')
    parser.add_argument('-p', '--processes_yaml_file',
                        default='$(find scrimmage_ros)/src/scrimmage_ros/test/templates/nav_2d_processes.yaml')
    parser.add_argument('-d', '--debug', action='store_true')

    # Parse the arguments
    args = parser.parse_args()

    # Specify a directory to hold logs and generated files
    logs_path = os.path.join(sru.user_home(), '.ros/scrimmage')
    run_dir = sru.make_get_run_dir(logs_path)

    entity_launch = EntityLaunch(args.sim_mission_yaml_file,
                                 args.processes_yaml_file,
                                 run_dir, None, None)

    if args.debug:
        entity_launch.print_processes()
        return

    # Run the processes. Blocking.
    entity_launch.run()

    return 0

if __name__ == '__main__':
    sys.exit(main())
