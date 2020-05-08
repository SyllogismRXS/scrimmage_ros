#!/usr/bin/env python

import os
import copy
import yaml
from jinja2 import Template

import scrimmage_ros.MultiProcessLogger as MPL
import scrimmage_ros.SimFilesGenerator as SFG
import scrimmage_ros.utils as sru

def render_id(string, entity_id):
    template = Template(string)
    return template.render(id=entity_id)

def get_name_command(process_info, entity_id):
    name = render_id(process_info['name'], entity_id)
    cmd = render_id(process_info['command'], entity_id)
    return name, cmd

def create_process(process_info, entity_id, output_dir, env, console, terminal):
    # Allow user to specify a post delay for a process
    try:
        post_delay = process_info['post_delay']
    except:
        post_delay = 0

    # Allow the user to append the environment with additional variables
    try:
        environment = { render_id(key, entity_id): render_id(value, entity_id)
                        for key, value in process_info['environment'].items() }
    except:
        environment = dict()

    # Make a deep copy of the environment to ensure that processes don't step
    # on each other's variables
    process_env = copy.deepcopy(env)
    process_env.update(environment)

    # Allow the user to change whether to write to the console or not
    try:
        console = process_info['console']
    except:
        # Don't override console type
        pass

    # Allow the user to change the terminal type
    try:
        terminal = MPL.Terminal(process_info['terminal'])
    except:
        # Use the default terminal
        pass

    # Perform template substitution based on entity_id {{ id }}
    name, command = get_name_command(process_info, entity_id)

    return {
        'command': command,
        'env': process_env,
        'console': console,
        'terminal': terminal,
        'file': output_dir + '/%s.log' % name,
        'post_delay': post_delay
    }

class EntityLaunch():
    def __init__(self, mission_yaml_filename, processess_yaml_file,
                 output_dir,  entity_ids=None, entity_type=None):
        self.entity_ids = entity_ids

        # The list of processes to pass to MultiProcessLogger
        self.processes = []
        self.clean_up_processes = []

        # Get the full path to the processes yaml file (ros find substitution)
        processes_yaml_file_path = sru.ros_find_path(processess_yaml_file)

        # Parse the processes file
        with open(processes_yaml_file_path) as f:
             self._processes_yaml = yaml.load(f, Loader=yaml.FullLoader)

        self.parse_defaults()

        # Get processes that are run before entities
        try:
            processes_list = self._processes_yaml['processes']
        except:
            processes_list = []

        # Append the processes that are run before entities
        for p in processes_list:
            self.processes.append(create_process(p, 0, output_dir, self.env, True, self.terminal))

        # If the entity_ids is none, use scrimmage to run a simulation
        if self.entity_ids is None:
            # Generate the scrimmage mission file and scrimmage file
            sfg = SFG.SimFilesGenerator(mission_yaml_filename, output_dir)

            # Get the entity IDs from the mission.yaml file
            self.entity_ids = sfg.entity_ids()

            # Append the scrimmage process
            self.processes.append(
                { 'command': "scrimmage %s" % sfg.mission_file_path,
                  'env': self.env,
                  'console': True,
                  'terminal': self.terminal,
                  'file': output_dir + '/scrimmage.log',
                }
            )

        # Create a mapping of entity type to process infos
        entity_type_to_processes = dict()
        for entity_type_processes in self._processes_yaml['entity_processes']:
            entity_type = entity_type_processes['type']
            entity_type_to_processes[entity_type] = entity_type_processes

        # Append the processes for each entity's roslaunch
        for entity_id in self.entity_ids:
            # Get the entity type:
            entity_type = sfg.entity_id_to_type(entity_id)

            # Get the entity processes for this type
            try:
                entity_processes = entity_type_to_processes[entity_type]['processes']
            except:
                entity_processes = []

            # Append the entity processes to the processes list
            for p in entity_processes:
                self.processes.append(create_process(p, entity_id, output_dir, self.env, True, self.terminal))

            # Get the clean up processes for this type
            try:
                entity_clean_up_processes = entity_type_to_processes[entity_type]['clean_up']
            except:
                entity_clean_up_processes = []

            # Append the clean up processes for this entity to the clean up list
            for p in entity_clean_up_processes:
                self.clean_up_processes.append(create_process(p, entity_id, output_dir, self.env, True, self.terminal))

        # Append the cleanup processes that apply to all entities
        try:
            clean_up_process_infos = self._processes_yaml['clean_up']
        except:
            clean_up_process_infos = []

        try:
            for p in clean_up_process_infos:
                self.clean_up_processes.append(create_process(p, 0, output_dir, self.env, True, self.terminal))
        except:
            pass

    def parse_defaults(self):
        # Parse the defaults block
        try:
            self.terminal = MPL.Terminal(self._processes_yaml['defaults']['terminal'])
        except:
            self.terminal=MPL.Terminal.gnome

        # Get environment variables to append to current environment
        try:
            environment = self._processes_yaml['defaults']['environment']
        except:
            environment = dict()

        # Environment setup
        self.env = os.environ.copy()
        self.env['HOME'] = sru.user_home()

        # Ensures that python prints and logs are displayed to screen
        self.env['PYTHONUNBUFFERED'] = '1'

        # Append user-defined environment variables
        self.env.update(environment)

    def print_processes(self):
        print('---------- Processes ------------')
        for p in self.processes:
            print(p['command'])

        print('---------- Clean up ------------')
        for p in self.clean_up_processes:
            print(p['command'])

    def print_environment(self):
        for p in self.processes:
            print("==================================================")
            print('Process: %s' % p['command'])
            print('### Environment ###')

            for key, value in p['env'].items():
                print('%s: %s' % (key, value))


    def run(self):
        mpl = MPL.MultiProcessLogger()

        # Run the processes. Blocking.
        mpl.run(self.processes, self.clean_up_processes)
