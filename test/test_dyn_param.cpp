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

#include <scrimmage/common/CSV.h>
#include <scrimmage/parse/ParseUtils.h>

#include <thread>
#include <chrono>
#include <vector>

#include <iostream>
using std::cout;
using std::endl;

namespace sc = scrimmage;

class DynParamTest : public testing::Test {
 protected:
    void SetUp() override {
    }
    ros::NodeHandle nh_;
    scrimmage_ros::dynamic_param_client param_client_;
 public:
};

TEST_F(DynParamTest, DynParamSet) {
    param_client_.init(nh_);

    // Update the list of SCRIMMAGE nodes using dynamic reconfigure
    uint64_t count = 0;
    while (param_client_.services().size() != 1 && count < 1e5) {
        ros::spinOnce(); // Process callbacks

        auto gen_configs = [](std::vector<scrimmage_ros::scrimmage_rosConfig> list) -> void {
            scrimmage_ros::scrimmage_rosConfig config;
            config.param_name = "random_variable";
            config.param_value = "12345";
            config.param_type = scrimmage_ros::scrimmage_ros_int;
            list.push_back(config);
        };
        // Note: We need a test for not passing in a generator, at a time when
        //      it does detect a new service.  Then, this test for passing in a
        //      generator, at a time when it does detect a new service.  The
        //      way this test is currently structured, launching other nodes
        //      without time delays, that's not possible.
        if (not param_client_.update_dynamic_param_servers(gen_configs)) {
            break;
        }
        ++count;
    }

    // There should only be one other scrimmage_ros node
    EXPECT_EQ(param_client_.services().size(), static_cast<unsigned int>(1));

    ///////////////////////////////////////////////////////////////////////////
    // Change each of the APITester plugin's dynamic plugin parameters
    ///////////////////////////////////////////////////////////////////////////
    // bool interface
    scrimmage_ros::scrimmage_rosConfig config_bool;
    config_bool.param_name = "my_test_bool";
    config_bool.param_value = "false";
    config_bool.param_type = scrimmage_ros::scrimmage_ros_bool;
    EXPECT_TRUE(param_client_.send_config(config_bool));

    // int interface
    scrimmage_ros::scrimmage_rosConfig config_int;
    config_int.param_name = "my_test_int";
    config_int.param_value = "2";
    config_int.param_type = scrimmage_ros::scrimmage_ros_int;
    EXPECT_TRUE(param_client_.send_config(config_int));

    // float interface
    scrimmage_ros::scrimmage_rosConfig config_float;
    config_float.param_name = "my_test_float";
    config_float.param_value = "3.2";
    config_float.param_type = scrimmage_ros::scrimmage_ros_float;
    EXPECT_TRUE(param_client_.send_config(config_float));

    // double interface
    scrimmage_ros::scrimmage_rosConfig config_double;
    config_double.param_name = "my_test_double";
    config_double.param_value = "98.987";
    config_double.param_type = scrimmage_ros::scrimmage_ros_double;
    EXPECT_TRUE(param_client_.send_config(config_double));

    ros::NodeHandle private_nh("~");
    int test_id = 0;
    if (not private_nh.getParam("test_id", test_id)) {
        ROS_ERROR_STREAM("Missing test_id.");
    }

    sc::CSV csv;
    EXPECT_TRUE(csv.read_csv(sc::expand_user("~/.scrimmage/logs/test"
                                             + std::to_string(test_id)
                                             + "/latest/api_tester.csv")));

    // Expected CSV output (one parameter changes in each row):
    sc::CSV expected_csv;
    expected_csv.read_csv_from_string(
        "my_test_bool,my_test_int,my_test_float,my_test_double\n"
        "1,1,2.100000,99.987000\n"
        "0,1,2.100000,99.987000\n"
        "0,2,2.100000,99.987000\n"
        "0,2,3.200000,99.987000\n"
        "0,2,3.200000,98.987000");

    bool equal = csv.equals(expected_csv);
    EXPECT_TRUE(equal);
    if (not equal) {
        std::cerr << "Expected the following CSV table: " << std::endl;
        std::cerr << expected_csv << endl;
        std::cerr << "But the actual CSV table is: " << std::endl;
        std::cerr << csv << endl;
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "dyn_param_test");
    return RUN_ALL_TESTS();
}
