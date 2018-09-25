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

#include <XmlRpcValue.h>
#include <time.h>

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

    std::ostringstream buf;
    sc_ros = std::make_shared<scrimmage_ros>(nh, private_nh, this->getName());
    sc_ros->init(buf);
    NODELET_INFO_STREAM(buf.str());

    // Call init() for subclasses
    if (!init()) {
        NODELET_ERROR_STREAM("init() call failed in " << this->getName());
    }

    // Start the callback step()
    loop_timer_ = nh.createTimer(ros::Duration(1.0/sc_ros->loop_rate_hz()),
                                 boost::bind(&Nodelet::timer_cb, this, _1));
}
void Nodelet::timer_cb(const ros::TimerEvent& event) {
    NODELET_INFO_STREAM("The time is now " << event.current_real);

    std::ostringstream buf;
    sc_ros->step(ros::Time::now().toSec(), buf);
    if (buf.str() != "") {
        NODELET_INFO_STREAM(buf.str());
    }

    // Call the subclass
    if (!step()) {
        NODELET_ERROR_STREAM("step() call failed in " << this->getName());
    }
}
} // namespace scrimmage_ros
