#!/usr/bin/env python

import os
import io
import sys
import subprocess
import signal
import time
import scrimmage_ros.utils as sru

from enum import Enum

from threading import Thread
from subprocess import Popen, PIPE, call

try:
    from queue import Queue, Empty
except:
    from Queue import Queue, Empty

class GracefulShutdown:
    shutdown_now = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.shutdown_now = True

class Terminal(Enum):
    none = 'none'
    gnome = 'gnome'
    tmux = 'tmux'
    screen = 'screen'

    def __str__(self):
        return self.value

class MultiProcessLogger():
    def __init__(self):
        self.terminal = Terminal
        pass

    def enqueue_logging(self, output, queue, process_number):
        while True:
            try:
                for line in io.TextIOWrapper(output, encoding='utf-8'):
                    queue.put((line, process_number))
                output.close()
            except:
                pass

    def run(self, process_info, clean_up_processes=[]):
        self.processes = []
        threads = []
        queue = Queue()

        start_tmux = False
        for pi in process_info:
            if 'terminal' in pi:
                if pi['terminal'] == self.terminal.tmux:
                    start_tmux = True
                    break

        if start_tmux:
            pipe = Popen('echo $TMUX_PANE', shell=True, stdout=PIPE)
            if (pipe.communicate()[0] == b'\n'):
                self.processes.append(Popen('tmux new -d',
                                        shell=True,
                                        stderr=PIPE, stdout=PIPE, bufsize=0))
                clean_up_processes.append({'command': 'tmux kill-server', \
                                        'env': process_info[0]['env'],
                                        'console': process_info[0]['console'],  \
                                        'terminal': process_info[0]['terminal'], \
                                        'file': '', \
                                        'post_delay': 0 })

        for i in range(len(process_info)):
            # Setup file descriptors for log files
            if 'file' in process_info[i]:
                sru.make_file_dirs(process_info[i]['file'])
                process_info[i]['fd'] = open(process_info[i]['file'], 'a')
            else:
                process_info[i]['file'] = '/dev/null'
                process_info[i]['fd'] = None

            cmd = process_info[i]['command']
            launch_file = process_info[i]['file']
            new_shell = False

            if 'terminal' in process_info[i]:
                if process_info[i]['terminal'] == self.terminal.gnome:
                    title = sru.gnome_terminal_title('my_window')
                    cmd = sru.gnome_terminal_cmd(title, cmd, launch_file)
                    new_shell = True
                elif process_info[i]['terminal'] == self.terminal.tmux:
                    options="-d "
                    for key in process_info[i]['env']:
                        value = process_info[i]['env'][key]
                        options = '{0} -e {1}="{2}"'.format(options, key, value)
                    cmd = sru.tmux_terminal_cmd(cmd, options, launch_file)
                    new_shell = True
                elif process_info[i]['terminal'] == self.terminal.screen:
                    cmd = "screen -d -m {0}".format(cmd)
                    new_shell = True
                else:
                    cmd = cmd.split()
            else:
                cmd = cmd.split()

            self.processes.append(Popen(cmd,
                                        env=process_info[i]['env'],
                                        shell=new_shell,
                                        stderr=PIPE, stdout=PIPE, bufsize=0))

            # If post_delay is specified, sleep for specified time before
            # launching next process
            if 'post_delay' in process_info[i]:
                time.sleep(process_info[i]['post_delay'])


        # Setup the threads that capture the output (stdout and stderr) from
        # each process and send the output to the logging queue
        for i in range(len(self.processes)):
            threads.append(Thread(target=self.enqueue_logging,
                                  args=(self.processes[i].stdout, queue, i)))
            threads.append(Thread(target=self.enqueue_logging,
                                  args=(self.processes[i].stderr, queue, i)))

        # Start the logging / queue threads
        for t in threads:
            t.daemon = True
            t.start()

        # Detect CTRL+c from the user
        graceful_shutdown = GracefulShutdown()

        # While waiting for all processes to complete or for the user to
        # shutdown the program, service the logging queue.
        while len(self.processes) > 0 and not graceful_shutdown.shutdown_now:
            self.processes = [p for p in self.processes if p.poll() is None]

            try:
                line = queue.get_nowait()

                # Write to the console if specified
                if process_info[line[1]]['console']:
                    sys.stdout.write(line[0])
                    sys.stdout.flush()

                # Write to the log file is specified
                if process_info[line[1]]['fd']:
                    process_info[line[1]]['fd'].write(line[0])
                    process_info[line[1]]['fd'].flush()
                    pass

            except Empty:
                pass

            time.sleep(0.001)

        shutdown_start = time.time()
        for p in self.processes:
            p.terminate()

        # Allow time for the processes to shutdown gracefully
        while len(self.processes) > 0:
            self.processes = [p for p in self.processes if p.poll() is None]
            time.sleep(0.25)
            print('Waiting for processes to exit.')
        shutdown_duration = time.time() - shutdown_start
        print('Time to shutdown = %f sec' % shutdown_duration)

        # Run the optional cleanup processes
        if len(clean_up_processes) > 0:
            print('Running cleanup processes...')
            for process in clean_up_processes:
                call(process['command'].split())
