import re
import rospkg
from jinja2 import FileSystemLoader, Environment

# returns full path to file where "my_string" is of the form:
# $(find <ros-package-name>)/path/to/file.txt
def ros_find_path(my_string):
    pattern = "\\$\\(find (.+)\\)"

    m = re.search(pattern, my_string)
    ros_package_name = m.group(1)

    # Construct the path to the file
    rospack = rospkg.RosPack()
    ros_package_path = rospack.get_path(ros_package_name)

    # Replace the $(find <package-name>) in the original string
    return re.sub(pattern, ros_package_path, my_string)

def generate_sim_roslaunch(config):
    # Load the jinja templates
    script_dir_path = os.path.dirname(os.path.realpath(__file__))

    file_loader = FileSystemLoader(os.path.join(script_dir_path, 'templates'))
    env = Environment(loader=file_loader)

    return render('sim.template.launch', config, env)

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

def gnome_terminal_cmd(title, cmd):
    return "gnome-terminal --disable-factory -x bash -c '" + cmd + "; exec bash'"
