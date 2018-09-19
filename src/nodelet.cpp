/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage_ros/nodelet.h>
#include "ros/console.h"

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/common/Algorithm.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/MissionParse.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(scrimmage_ros::Nodelet, nodelet::Nodelet)

namespace sc = scrimmage;
namespace fs = boost::filesystem;

namespace scrimmage_ros {
Nodelet::Nodelet() {
}
void Nodelet::onInit() {
    // Get ROS node handles
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle private_nh = getPrivateNodeHandle();

    // Get ros parameters
    double loop_rate_hz = 5;
    if (not private_nh.getParam("loop_rate_hz", loop_rate_hz)) {
        NODELET_WARN_STREAM(this->getName() << " missing ros param: loop_rate_hz.");
    }

    std::string mission_file;
    if (not private_nh.getParam("mission_file", mission_file)) {
        NODELET_ERROR_STREAM(this->getName() << "missing ros param: mission_file.");
    }

    std::string entity_name;
    if (not private_nh.getParam("entity_name", entity_name)) {
        NODELET_ERROR_STREAM(this->getName() << "missing ros param: entity_name.");
    }

    std::string plugin_tags_str;
    if (not private_nh.getParam("tags", plugin_tags_str)) {
        NODELET_ERROR_STREAM(this->getName() << "missing ros param: plugin_tags.");
    }

    if (not private_nh.getParam("entity_id", entity_id_)) {
        NODELET_WARN_STREAM(this->getName() << "missing ros param: entity_id.");
    }

    int max_contacts = 100;
    if (not private_nh.getParam("max_contacts", max_contacts)) {
        NODELET_WARN_STREAM(this->getName() << "missing ros param: max_contacts.");
    }

    // Call init() for subclasses
    if (!init()) {
        NODELET_ERROR_STREAM("init() call failed in " << this->getName());
    }

    // Get the current ROS log directory
    ros_log_dir_ = sc::exec_command("roslaunch-logs");
    ros_log_dir_.erase(std::remove(ros_log_dir_.begin(), ros_log_dir_.end(), '\n'), ros_log_dir_.end());
    if (not fs::exists(fs::path(ros_log_dir_))) {
        NODELET_ERROR_STREAM("ROS log directory doesn't exist: " << ros_log_dir_);
    }

    const bool create_entity =
        external_.create_entity(mission_file, entity_name, plugin_tags_str,
                                entity_id_, max_contacts,
                                ros_log_dir_ + "/scrimmage");
    if (create_entity) {
        std::ostringstream buf;
        external_.print_plugins(buf);
        NODELET_INFO_STREAM(buf.str());
    } else {
        NODELET_ERROR_STREAM(this->getName() << "failed to load plugins for " << entity_name);
    }

    // Start the callback step()
    loop_timer_ = nh.createTimer(ros::Duration(1.0/loop_rate_hz), boost::bind(&Nodelet::timer_cb, this, _1));

}
void Nodelet::timer_cb(const ros::TimerEvent& event) {
    NODELET_INFO_STREAM("The time is now " << event.current_real);

    external_.step(ros::Time::now().toSec());

    if (!step()) {
        NODELET_ERROR_STREAM("step() call failed in " << this->getName());
    }
}
} // namespace scrimmage_ros
