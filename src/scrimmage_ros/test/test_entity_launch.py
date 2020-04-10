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
    parser.add_argument('-p', '--ros_package_name', type=str,
                        default='scrimmage_ros',
                        help='ROS package that contains launch file')
    parser.add_argument('-f', '--launch_file', type=str,
                        default='entity_nav_2d.launch',
                        help='Launch file in ROS package')
    parser.add_argument('-t', '--terminal', type=Terminal,
                        choices=list(Terminal),
                        default=Terminal.gnome)

    # Parse the arguments
    args = parser.parse_args()

    # ROS launch args. {{ id }} is substituted for entity ID
    launch_args = 'team_id:=1 entity_id:={{ id }}'

    # Specify a directory to hold logs and generated files
    logs_path = os.path.join(sru.user_home(), '.ros/scrimmage')
    run_dir = sru.make_get_run_dir(logs_path)

    entity_launch = EntityLaunch(args.sim_mission_yaml_file, run_dir,
                                 args.ros_package_name, args.launch_file,
                                 launch_args, None, args.terminal)

    # Append a custom ros launch file to the processes list to launch a common
    # map server for both agents. The map server exists outside of a ROS
    # namespace
    entity_launch.processes.append(
        { 'command': sru.ros_launch('scrimmage_ros', 'map_server.launch', ''),
          'env': os.environ.copy(),
          'console': False,
          'terminal': Terminal.gnome
        }
    )

    # Run the processes. Blocking.
    entity_launch.run()

    return 0

if __name__ == '__main__':
    sys.exit(main())
