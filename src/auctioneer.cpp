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

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/External.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/msgs/AuctionMsgs.pb.h>

#include <scrimmage_ros/RosBidAuction.h>
#include <scrimmage_ros/RosStartAuction.h>

#if ENABLE_PYTHON_BINDINGS == 1
#include <Python.h>
#endif

namespace sc = scrimmage;
using scrimmage_ros::RosBidAuction;
using scrimmage_ros::RosStartAuction;

sc::Message<auction::StartAuction> ros2sc_start_auction(const RosStartAuction &ros_msg) {
    sc::Message<auction::StartAuction> sc_msg;
    sc_msg.data.set_sender_id(ros_msg.sender_id);
    return sc_msg;
}

sc::Message<auction::BidAuction> ros2sc_bid_auction(const RosBidAuction &ros_msg) {
    sc::Message<auction::BidAuction> sc_msg;
    sc_msg.data.set_bid(ros_msg.bid);
    sc_msg.data.set_sender_id(ros_msg.sender_id);
    return sc_msg;
}

RosStartAuction sc2ros_start_auction(const std::shared_ptr<sc::Message<auction::StartAuction>> &sc_msg) {
    RosStartAuction ros_msg;
    ros_msg.sender_id = sc_msg->data.sender_id();
    return ros_msg;
}

RosBidAuction sc2ros_bid_auction(const std::shared_ptr<sc::Message<auction::BidAuction>> &sc_msg) {
    RosBidAuction ros_msg;
    ros_msg.sender_id = sc_msg->data.sender_id();
    ros_msg.bid = sc_msg->data.bid();
    return ros_msg;
}

int main(int argc, char **argv) {
#if ENABLE_PYTHON_BINDINGS == 1
    Py_Initialize();
#endif

    // Initialize the ROS node
    ros::init(argc, argv, "auctioneer");
    ros::NodeHandle nh;

    // Get a private node handle to parse ros params
    ros::NodeHandle private_nh("~");

    // Get the mission file, which holds overrides for plugin parameters
    std::string mission_file;
    private_nh.param("mission_file", mission_file, std::string(""));

    // Get the entity ID
    int entity_id;
    private_nh.param("entity_id", entity_id, 1);

    // Get the entity name. This name should match the <name> tag in the
    // mission file for the entity block we want to use.
    std::string entity_name;
    private_nh.param("entity_name", entity_name, std::string("UNDEFINED"));

    // Specify the maximum number of contacts that we will encounter.
    int max_contacts;
    private_nh.param("max_contacts", max_contacts, 100);

    sc::External external;
    external.create_entity(mission_file, max_contacts, entity_id, entity_name);

    ros::Publisher pub_start_auction =
        nh.advertise<RosStartAuction>("StartAuction", 1000);
    external.pub_cb<auction::StartAuction>("SphereNetwork", "StartAuction",
                                         sc2ros_start_auction, pub_start_auction);

    ros::Publisher pub_bid_auction =
        nh.advertise<RosBidAuction>("BidAuction", 1000);
    external.pub_cb<auction::BidAuction>("SphereNetwork", "BidAuction",
                                         sc2ros_bid_auction, pub_bid_auction);

    ros::Subscriber sub_start_auction = nh.subscribe("StartAuction", 1000,
        external.sub_cb<RosStartAuction>("SphereNetwork", "StartAuction",
                                         ros2sc_start_auction));

    ros::Subscriber sub_bid_auction = nh.subscribe("BidAuction", 1000,
        external.sub_cb<RosBidAuction>("SphereNetwork", "BidAuction",
                                       ros2sc_bid_auction));

    const double loop_rate_hz = 10;
    const double startup_delay = 1;
    const double runtime = 10;
    ros::Rate loop_rate(loop_rate_hz);

    // wait for all nodes to startup
    for (int i = 0; i < startup_delay * loop_rate_hz; i++) {
        loop_rate.sleep();
    }

    int count = 0;
    while (ros::ok() && count < runtime * loop_rate_hz) {
        external.step(ros::Time::now().toSec());
        loop_rate.sleep();
        ros::spinOnce();
        ++count;
    }

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif

    return 0;
}
