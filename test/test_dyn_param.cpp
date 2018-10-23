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
#include <ros/ros.h>
#include <ros/time.h>
#include <gtest/gtest.h>

#include <scrimmage_ros/scrimmage_rosConfig.h>
#include <scrimmage_ros/dynamic_param_client.h>

#include <thread>
#include <chrono>

#include <iostream>
using std::cout;
using std::endl;

class DynParamTest : public testing::Test {
 protected:
    void SetUp() override {
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh_;
    }
    scrimmage_ros::dynamic_param_client param_client_;
 public:
};

TEST_F(DynParamTest, DynParamSet) {
    // Update the list of SCRIMMAGE nodes using dynamic reconfigure
    uint64_t count = 0;
    while (param_client_.services().size() != 1 && count < 1e5) {
        if (not param_client_.update_dynamic_param_servers()) {
            break;
        }
        ++count;
    }

    // There should only be one other scrimmage_ros node
    EXPECT_EQ(param_client_.services().size(), 1);

    // Create a configuration to send
    scrimmage_ros::scrimmage_rosConfig config;
    config.param_name = "max_speed";
    config.param_value = "15.023";
    config.param_type = scrimmage_ros::scrimmage_ros_double;

    // Send the configuration
    EXPECT_TRUE(param_client_.send_config(config));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "dyn_param_test");
    ros::start();
    return RUN_ALL_TESTS();
}
