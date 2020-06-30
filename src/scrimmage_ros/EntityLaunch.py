#!/usr/bin/env python

import os
import copy
import yaml
from jinja2 import Template

import scrimmage_ros.MultiProcessLogger as MPL
import scrimmage_ros.SimFilesGenerator as SFG
import scrimmage_ros.utils as sru

def render_variables(string, entity_id, base_logs_path, run_dir, process_dir):
    template = Template(str(string))
    return template.render(id=entity_id, base_logs_path=base_logs_path,
                           run_dir=run_dir, process_dir=process_dir)

def get_name_command(process_info, entity_id, base_logs_path, run_dir, process_dir):
    name = render_variables(process_info['name'], entity_id, base_logs_path, run_dir, process_dir)
    cmd = render_variables(process_info['command'], entity_id, base_logs_path, run_dir, process_dir)
    return name, cmd

def create_process(process_info, entity_id, base_logs_path, run_dir, env, console, terminal):
    # Allow user to specify a post delay for a process
    try:
        post_delay = process_info['post_delay']
    except:
        post_delay = 0

    process_dir = run_dir + '/entity%d' % entity_id

    # Allow the user to append the environment with additional variables
    try:
        environment = { render_variables(key, entity_id, base_logs_path, run_dir, process_dir):
                        render_variables(value, entity_id, base_logs_path, run_dir, process_dir)
                        for key, value in process_info['environment'].items() }
    except KeyError:
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
    name, command = get_name_command(process_info, entity_id, base_logs_path,
                                     run_dir, process_dir)

    return {
        'command': command,
        'env': process_env,
        'console': console,
        'terminal': terminal,
        'file': process_dir + '/%s.log' % name,
        'post_delay': post_delay
    }

class EntityLaunch():
    def __init__(self, mission_yaml_filename, processess_yaml_file,
                 base_logs_path, run_dir, entity_id=None, entity_type=None):

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
            self.processes.append(create_process(p, 0, base_logs_path, run_dir,
                                                 self.env, True,
                                                 self.terminal))

        # Create a mapping of entity type to process infos
        entity_type_to_processes = self._get_entity_type_to_processes()

        # If the entity_ids is none, use scrimmage to run a simulation
        if entity_id is None:
            # Generate the scrimmage mission file and scrimmage file
            sfg = SFG.SimFilesGenerator(mission_yaml_filename, run_dir)

            # Get the entity IDs from the mission.yaml file
            self.entity_ids = sfg.entity_ids()

            # Append the scrimmage process
            self.processes.append(
                { 'command': "scrimmage %s" % sfg.mission_file_path,
                  'env': self.env,
                  'console': True,
                  'terminal': self.terminal,
                  'file': run_dir + '/scrimmage.log',
                }
            )
        else:
            # If the entity_id is specified, only use this ID, regardless of
            # what is listed in the mission yaml file.
            self.entity_ids = [ entity_id ]

        # Append the processes for each entity's roslaunch
        for entity_id in self.entity_ids:
            # If entity_type is not provided, use the entity_type in the yaml
            # files. Otherwise, we use the manually specified type.
            if entity_type is None:
                try:
                    entity_type = sfg.entity_id_to_type(entity_id)
                except:
                    raise NameError('You must specify the entity_type when specifying the entity_id')

            # Get the entity processes for this type
            try:
                entity_processes = entity_type_to_processes[entity_type]['processes']
            except:
                entity_processes = []

            # Append the entity processes to the processes list
            for p in entity_processes:
                self.processes.append(
                    create_process(p, entity_id, base_logs_path, run_dir,
                                   self.env, True, self.terminal))

            # Get the clean up processes for this type
            try:
                entity_clean_up_processes = entity_type_to_processes[entity_type]['clean_up']
            except:
                entity_clean_up_processes = []

            # Append the clean up processes for this entity to the clean up list
            for p in entity_clean_up_processes:
                self.clean_up_processes.append(
                    create_process(p, entity_id, base_logs_path, run_dir,
                                   self.env, True, self.terminal))

        # Append the cleanup processes that apply to all entities
        try:
            clean_up_process_infos = self._processes_yaml['clean_up']
        except:
            clean_up_process_infos = []

        try:
            for p in clean_up_process_infos:
                self.clean_up_processes.append(
                    create_process(p, 0, base_logs_path, run_dir, self.env,
                                   True, self.terminal))
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

    def _get_entity_type_to_processes(self):
        entity_type_to_processes = dict()
        for entity_type_processes in self._processes_yaml['entity_processes']:
            entity_type = entity_type_processes['type']
            entity_type_to_processes[entity_type] = entity_type_processes
        return entity_type_to_processes

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
