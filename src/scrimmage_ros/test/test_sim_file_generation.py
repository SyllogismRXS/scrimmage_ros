#!/usr/bin/env python

import os
import scrimmage_ros.SimFilesGenerator as MG
import tempfile
import shutil

with tempfile.TemporaryDirectory() as tmp_dir_name:
    mission_yaml = '$(find scrimmage_ros)/missions/example_template/mission.yaml'
    sfg = MG.SimFilesGenerator(mission_yaml, tmp_dir_name)

    print("==================================================================")
    print("SCRIMMAGE MISSION File: %s" % sfg.mission_file_path)
    print("==================================================================")
    with open(sfg.mission_file_path, 'r') as file:
        print(file.read())
