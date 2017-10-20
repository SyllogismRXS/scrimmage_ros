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
#include <std_msgs/Int16.h>

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/External.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/AuctionMsgs.pb.h>

#include <scrimmage_ros/RosBidAuction.h>

#if ENABLE_PYTHON_BINDINGS == 1
#include <Python.h>
#endif

namespace sc = scrimmage;
using BidMsg = sc::Message<auction::BidAuction>;
using scrimmage_ros::RosBidAuction;

sc::MessageBase ros2sc_start_auction(const std_msgs::Int16 &ros_msg) {
    sc::MessageBase sc_msg;
    sc_msg.sender = ros_msg.data;
    return sc_msg;
}

std_msgs::Int16 sc2ros_start_auction(const sc::MessageBasePtr &sc_msg) {
    std_msgs::Int16 ros_msg;
    ros_msg.data = sc_msg->sender;
    return ros_msg;
}

BidMsg ros2sc_bid_auction(const RosBidAuction &ros_msg) {
    BidMsg sc_msg;
    sc_msg.sender = ros_msg.id;
    sc_msg.data.set_bid(ros_msg.bid);
    return sc_msg;
}

RosBidAuction sc2ros_bid_auction(const std::shared_ptr<BidMsg> &sc_msg) {
    RosBidAuction ros_msg;
    ros_msg.id = sc_msg->sender;
    ros_msg.bid = sc_msg->data.bid();
    return ros_msg;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "need an argument for id" << std::endl;
        return 1;
    }

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Initialize();
#endif

    sc::ID id(std::stoi(argv[1]), 0, 0);
    ros::init(argc, argv, "auctioneer");

    ros::NodeHandle nh;

    std::map<std::string, std::string> info
        {{"x", "0"}, {"y", "0"}, {"autonomy0", "AuctionAssign"}};

    sc::External external;
    external.create_entity(100, id, info, "/tmp");

    auto pubs = external.entity()->autonomies().front()->pubs();

    ros::Publisher pub_start_auction =
        nh.advertise<std_msgs::Int16>("StartAuction", 1000);
    external.pub_cb(sc2ros_start_auction,
        pubs["StartAuction"], pub_start_auction);

    ros::Publisher pub_bid_auction =
        nh.advertise<RosBidAuction>("BidAuction", 1000);
    external.pub_cb<auction::BidAuction>(sc2ros_bid_auction,
        pubs["BidAuction"], pub_bid_auction);

    auto subs = external.entity()->autonomies().front()->subs();

    ros::Subscriber sub_start_auction = nh.subscribe("StartAuction", 1000,
        external.sub_cb<std_msgs::Int16>(ros2sc_start_auction, subs["StartAuction"]));

    ros::Subscriber sub_bid_auction = nh.subscribe("BidAuction", 1000,
        external.sub_cb<RosBidAuction>(ros2sc_bid_auction, subs["BidAuction"]));

    const double loop_rate_hz = 10;
    const double startup_delay = 1;
    const double runtime = 10;
    ros::Rate loop_rate(loop_rate_hz);

    // wait for all nodes to startup
    for (int i = 0; i < startup_delay * loop_rate_hz; i++) {
        loop_rate.sleep();
    }

    int ct = 0;
    while (ros::ok() && ct < runtime * loop_rate_hz) {
        external.step(ros::Time::now().toSec());
        loop_rate.sleep();
        ros::spinOnce();
        ++ct;
    }

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif

    return 0;
}
