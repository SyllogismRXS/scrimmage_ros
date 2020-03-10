#!/usr/bin/env python

import os
import tempfile
import shutil

import scrimmage_ros.utils as sru
import scrimmage_ros.MultiProcessLogger as MPL
import scrimmage_ros.SimFilesGenerator as SFG

# The list of processes to pass to MultiProcessLogger
processes = []

mpl = MPL.MultiProcessLogger()

env = os.environ.copy()

tmp_dir = tempfile.mkdtemp()

# Generate the scrimmage mission file and simulation roslaunch file
sfg = SFG.SimFilesGenerator('$(find scrimmage_ros)/missions/example_template/mission.yaml', tmp_dir)

# Get the entity IDs from the mission.yaml file
entity_ids = sfg.entity_ids()

# Create the process that launches scrimmage and sets simulation ROS params.
processes.append(
    { 'command': sru.ros_launch_file(sfg.launch_file_path),
      'env': env,
      'console': True,
      'file': tmp_dir + '/roslaunch.log',
      'post_delay': 1.5
    }
)

for entity_id in range(1, 3):
    args = 'team_id:=1 entity_id:=%d' % entity_id
    processes.append(
        { 'command': sru.ros_launch('scrimmage_ros', 'test_entity.launch', args),
          'env': env,
          'console': True,
          'terminal': mpl.terminal.gnome,
          'file': tmp_dir + '/entity%d/entity.log' % entity_id
        }
    )

# Run the processes
mpl.run(processes)

# Remove the temporary files
shutil.rmtree(tmp_dir)
