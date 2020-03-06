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
