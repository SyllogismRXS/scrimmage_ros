import os
import re
import rospkg
from jinja2 import FileSystemLoader, Environment

# returns full path to file where "my_string" is of the form:
# $(find <ros-package-name>)/path/to/file.txt
def ros_find_path(my_string):
    pattern = "\\$\\(find (.+)\\)"

    m = re.search(pattern, my_string)
    try:
        ros_package_name = m.group(1)
    except:
        # If the group doesn't exist, there isn't anything to replace, just
        # return the input string
        return my_string

    # Construct the path to the file
    rospack = rospkg.RosPack()
    ros_package_path = rospack.get_path(ros_package_name)

    # Replace the $(find <package-name>) in the original string
    return re.sub(pattern, ros_package_path, my_string)

def render(template_name, config, env):
    template = env.get_template(template_name)
    return template.render(config = config)

def generate_sim_roslaunch(config):
    # Load the jinja templates
    script_dir_path = os.path.dirname(os.path.realpath(__file__))

    file_loader = FileSystemLoader(os.path.join(script_dir_path, 'templates'))
    env = Environment(loader=file_loader)

    template = env.get_template('sim.template.launch')
    return template.render(config = config)

# Given a file, create the directory structure for that file to be created
def make_file_dirs(file_path):
    if not os.path.exists(os.path.dirname(file_path)):
        try:
            os.makedirs(os.path.dirname(file_path))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

# Given a directory, create the directory structure required
def make_dirs(directory):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

def ros_launch(package, launch_file, args=''):
    return 'stdbuf -oL roslaunch ' + package + ' ' + launch_file + ' ' + args

def ros_launch_file(launch_file, args=''):
    return 'stdbuf -oL roslaunch ' + launch_file + ' ' + args

def gnome_terminal_title(title):
    return "echo -ne \\\"\\033]0;" + title + "\\007\\\""

def gnome_terminal_cmd(title, cmd, log_file):
    return "gnome-terminal --disable-factory -- bash -c '" + cmd \
        + " 2>&1 | tee " + log_file + "; exec bash'"

def tmux_terminal_cmd(cmd, options, log_file):
    return "tmux new-window " + options +  " '" + cmd  \
        + " 2>&1 | tee " + log_file + "; exec bash'"

def user_home():
    # The HOME variable doesn't exist if running as root
    try:
        user_home = os.environ['HOME']
    except KeyError:
        user_home = '/opt'
    return user_home

def make_get_run_dir(base_logs_path):
    make_dirs(base_logs_path)

    # Determine the next runXXX directory and create it
    run_num = 0
    while os.path.exists(base_logs_path + "/run%03d" % run_num):
        run_num += 1

    log_run_dir = base_logs_path + "/run%03d" % run_num
    make_dirs(log_run_dir)

    # Create the symlink to the latest run directory
    latest_symlink = base_logs_path + "/latest"
    if os.path.islink(latest_symlink):
        os.remove(latest_symlink)
    os.symlink(log_run_dir, latest_symlink)

    return log_run_dir
