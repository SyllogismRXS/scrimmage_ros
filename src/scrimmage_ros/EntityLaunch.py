#!/usr/bin/env python

import os
from jinja2 import Template

import scrimmage_ros.MultiProcessLogger as MPL
import scrimmage_ros.SimFilesGenerator as SFG
import scrimmage_ros.utils as sru

class EntityLaunch():
    def __init__(self, mission_yaml_filename, output_dir, ros_package_name,
                 launch_filename, launch_args='team_id:=1 entity_id:={{ id }}',
                 entity_ids=None, terminal=MPL.Terminal.gnome):
        # The list of processes to pass to MultiProcessLogger
        self.processes = []

        self._mpl = MPL.MultiProcessLogger()

        env = os.environ.copy()

        env['HOME'] = sru.user_home()

        # Append the roscore process
        self.processes.append(
            { 'command': "roscore",
              'env': env,
              'console': True,
              'file': output_dir + '/roscore.log',
              'post_delay': 1.5
            }
        )

        if entity_ids is None:
            # Generate the scrimmage mission file and scrimmage file
            sfg = SFG.SimFilesGenerator(mission_yaml_filename, output_dir)

            # Get the entity IDs from the mission.yaml file
            entity_ids = sfg.entity_ids()

            # Append the scrimmage process
            self.processes.append(
                { 'command': "scrimmage %s" % sfg.mission_file_path,
                  'env': env,
                  'console': True,
                  'terminal': terminal,
                  'file': output_dir + '/scrimmage.log',
                }
            )

        # Append the processes for each entity's roslaunch
        for entity_id in entity_ids:
            t = Template(launch_args)
            args = t.render(id=entity_id)

            self.processes.append(
                { 'command': sru.ros_launch(ros_package_name, launch_filename, args),
                  'env': env,
                  'console': True,
                  'terminal': terminal,
                  'file': output_dir + '/entity%d/entity.log' % entity_id
                }
            )

    def run(self):
        # Run the processes. Blocking.
        self._mpl.run(self.processes)
