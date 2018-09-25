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
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/msgs/AuctionMsgs.pb.h>

#include <scrimmage_ros/RosBidAuction.h>
#include <scrimmage_ros/RosStartAuction.h>
#include <scrimmage_ros/scrimmage_ros.h>

#if ENABLE_PYTHON_BINDINGS == 1
#include <Python.h>
#endif

namespace sc = scrimmage;
using scrimmage_ros::RosBidAuction;
using scrimmage_ros::RosStartAuction;

auction::StartAuction ros2sc_start_auction(const RosStartAuction &ros_msg) {
    auction::StartAuction sc_msg;
    sc_msg.set_sender_id(ros_msg.sender_id);
    return sc_msg;
}

auction::BidAuction ros2sc_bid_auction(const RosBidAuction &ros_msg) {
    auction::BidAuction sc_msg;
    sc_msg.set_bid(ros_msg.bid);
    sc_msg.set_sender_id(ros_msg.sender_id);
    return sc_msg;
}

RosStartAuction sc2ros_start_auction(const auction::StartAuction &sc_msg) {
    RosStartAuction ros_msg;
    ros_msg.sender_id = sc_msg.sender_id();
    return ros_msg;
}

RosBidAuction sc2ros_bid_auction(const auction::BidAuction &sc_msg) {
    RosBidAuction ros_msg;
    ros_msg.sender_id = sc_msg.sender_id();
    ros_msg.bid = sc_msg.bid();
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

    scrimmage_ros::scrimmage_ros sc_ros(nh, private_nh, ros::this_node::getName());
    std::ostringstream buf;
    sc_ros.init(buf);
    ROS_INFO_STREAM(buf.str());

    ros::Publisher pub_start_auction;
    ros::Publisher pub_bid_auction;
    ros::Publisher pub_result_auction;

    ros::Subscriber sub_start_auction;
    ros::Subscriber sub_bid_auction;

    const std::string network_name = "CommsNetwork";

    pub_start_auction = nh.advertise<RosStartAuction>("StartAuction", 1000);
    sc_ros.external().pub_cb<auction::StartAuction>(
        network_name, "StartAuction", sc2ros_start_auction, pub_start_auction);

    pub_bid_auction = nh.advertise<RosBidAuction>("BidAuction", 1000);
    sc_ros.external().pub_cb<auction::BidAuction>(
        network_name, "BidAuction", sc2ros_bid_auction, pub_bid_auction);

    sub_start_auction = nh.subscribe("StartAuction", 1000,
                                     sc_ros.external().sub_cb<RosStartAuction>(
                                         network_name, "StartAuction", ros2sc_start_auction));

    sub_bid_auction = nh.subscribe("BidAuction", 1000,
                                   sc_ros.external().sub_cb<RosBidAuction>(
                                       network_name, "BidAuction", ros2sc_bid_auction));

    pub_result_auction = nh.advertise<RosBidAuction>("ResultAuction", 1000);
    sc_ros.external().pub_cb<auction::BidAuction>(
        network_name, "ResultAuction", sc2ros_bid_auction, pub_result_auction);

    const double startup_delay = 1;
    const double runtime = 10;
    ros::Rate loop_rate(sc_ros.loop_rate_hz());

    // wait for all nodes to startup
    for (int i = 0; i < startup_delay * sc_ros.loop_rate_hz(); i++) {
        loop_rate.sleep();
    }

    int count = 0;
    while (ros::ok() && count++ < runtime * sc_ros.loop_rate_hz()) {
        sc_ros.external().step(ros::Time::now().toSec());
        loop_rate.sleep();
        ros::spinOnce();
    }

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif

    return 0;
}
