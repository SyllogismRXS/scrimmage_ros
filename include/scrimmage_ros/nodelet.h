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

#ifndef SCRIMMAGE_ROS_NODELET_H_
#define SCRIMMAGE_ROS_NODELET_H_

#include <scrimmage/entity/External.h>

#include <scrimmage_ros/scrimmage_ros.h>

#include "ros/ros.h"
#include "nodelet/nodelet.h"

namespace scrimmage_ros {

class Nodelet : public nodelet::Nodelet {
 public:
    Nodelet();

 protected:
    virtual bool init() { return true; }
    virtual bool step() { return true; }

    std::shared_ptr<scrimmage_ros> sc_ros;

private:
    void onInit() override;
    void timer_cb();
    ros::Timer loop_timer_;
};

} // namespace scrimmage_ros

#endif // SCRIMMAGE_ROS_SCRIMMAGE_NODELET_H_
