#!/usr/bin/env python

import os
import scrimmage_ros.utils as sru

print(sru.ros_find_path("$(find scrimmage_ros)/launch/auction.launch"))
