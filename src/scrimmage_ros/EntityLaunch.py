#!/usr/bin/env python

import os
from jinja2 import Template

import scrimmage_ros.MultiProcessLogger as MPL
import scrimmage_ros.SimFilesGenerator as SFG
import scrimmage_ros.utils as sru

def get_name_command(process, entity_id):
    name_t = Template(process['name'])
    name = name_t.render(id=entity_id)

    cmd_t = Template(process['command'])
    cmd = cmd_t.render(id=entity_id)

    return name, cmd

class EntityLaunch():
    def __init__(self, mission_yaml_filename, output_dir, ros_package_name,
                 launch_filename, launch_args='team_id:=1 entity_id:={{ id }}',
                 entity_ids=None, terminal=MPL.Terminal.gnome,
                 processes_per_entity=[], clean_up_processes_per_entity=[],
                 clean_up_processes=[]):
        # The list of processes to pass to MultiProcessLogger
        self.processes = []
        self.clean_up_processes = []

        self._mpl = MPL.MultiProcessLogger()

        env = os.environ.copy()

        env['HOME'] = sru.user_home()

        # Ensures that python prints and logs are displayed to screen
        env['PYTHONUNBUFFERED'] = '1'

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
            self.entity_ids = sfg.entity_ids()

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
        for entity_id in self.entity_ids:
            args_t = Template(launch_args)
            args = args_t.render(id=entity_id)

            self.processes.append(
                { 'command': sru.ros_launch(ros_package_name, launch_filename, args),
                  'env': env,
                  'console': True,
                  'terminal': terminal,
                  'file': output_dir + '/entity%d/entity.log' % entity_id
                }
            )

            # The user can pass in additional processes to run per entity
            for process in processes_per_entity:
                name, cmd = get_name_command(process, entity_id)

                self.processes.append(
                    { 'command': cmd,
                      'env': env,
                      'console': True,
                      'terminal': terminal,
                      'file': output_dir + '/entity%d/%s.log' % (entity_id, name)
                    }
                )

            # Append the cleanup processes that are associated with each entity
            for process in clean_up_processes_per_entity:
                name, cmd = get_name_command(process, entity_id)
                self.clean_up_processes.append(
                    {
                        'command': cmd,
                        'env': env,
                    }
                )

        # Append the cleanup processes that apply to all entities
        for process in clean_up_processes:
            name, cmd = get_name_command(process, 0)
            self.clean_up_processes.append(
                {
                    'command': cmd,
                    'env': env,
                }
            )


    def run(self):
        # Run the processes. Blocking.
        self._mpl.run(self.processes, self.clean_up_processes)
