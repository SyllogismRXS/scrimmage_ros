#!/usr/bin/env python

import os
import tempfile
import shutil

import scrimmage_ros.utils as sru
import scrimmage_ros.MultiProcessLogger as MPL

# The list of processes to pass to MultiProcessLogger
processes = []

mpl = MPL.MultiProcessLogger()

env = os.environ.copy()
env['PYTHONUNBUFFERED'] = '1'

tmp_dir = tempfile.mkdtemp()

processes.append(
    { 'command': 'roscore',
      'env': env,
      'console': True,
      'terminal': mpl.terminal.none,
      'file': tmp_dir + '/roscore.log',
      'post_delay': 1.5
    }
)

for id in range(1, 5):
    cmd = 'echo "Hello, Entity %d"' % id
    processes.append(
        { 'command': cmd,
          'env': env,
          'console': True,
          'terminal': mpl.terminal.none,
          'file': tmp_dir + '/entity%d/entity.log' % id
        }
    )

# Run the processes
mpl.run(processes)

# Remove the temporary files
shutil.rmtree(tmp_dir)
