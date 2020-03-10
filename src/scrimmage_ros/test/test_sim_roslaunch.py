#!/usr/bin/env python

import os
import scrimmage_ros.utils as sru

config = {'mission_file': 'some.xml'}

print(sru.generate_sim_roslaunch(config))
