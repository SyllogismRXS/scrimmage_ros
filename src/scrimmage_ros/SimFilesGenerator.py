"""Generates and writes the launch and SCRIMMAGE mission files from a YAML
   Configuration

@file

@section LICENSE

Copyright (C) 2020 by the Georgia Tech Research Institute (GTRI)

This file is part of SCRIMMAGE.

  SCRIMMAGE is free software: you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.

@author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
@author Eric Squires <eric.squires@gtri.gatech.edu>
@date 31 July 2017
@version 0.1.0
@brief Brief file description.
@section DESCRIPTION
A Long description goes here.

"""

import os

import scrimmage_ros.utils

import scrimmage.utils
import scrimmage.mission.MissionGenerator as MG

class SimFilesGenerator():
    def __init__(self, mission_yaml_file, output_dir):
        yaml_full_path = scrimmage_ros.utils.ros_find_path(mission_yaml_file)

        self._mission_gen = MG.MissionGenerator(yaml_full_path)

        sim_dir = output_dir + '/sim'
        scrimmage.utils.make_dir_tree(sim_dir)

        self.mission_file_path = os.path.join(sim_dir, 'temp_mission.xml')
        self.launch_file_path = os.path.join(sim_dir, 'sim.launch')

        with open(self.mission_file_path, 'w') as f:
            f.write(self._mission_gen.mission)

        with open(self.launch_file_path, 'w') as f:
            config = {'mission_file' : self.mission_file_path}
            f.write(scrimmage_ros.utils.generate_sim_roslaunch(config))

    def entity_ids(self):
        return self._mission_gen.entity_ids()
