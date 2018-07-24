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

#include <scrimmage_ros/RosStartAuction.h>
#include <scrimmage_ros/RosBidAuction.h>

using scrimmage_ros::RosStartAuction;
using scrimmage_ros::RosBidAuction;

class AuctionTest : public testing::Test {
 protected:
    virtual void SetUp() {
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh_;
        sub_start_ =
          nh_.subscribe("StartAuction", 20,
                &AuctionTest::start_auction_callback, this);
        sub_bid_ =
          nh_.subscribe("BidAuction", 20,
                &AuctionTest::bid_auction_callback, this);

        sub_result_ =
          nh_.subscribe("ResultAuction", 20,
                &AuctionTest::result_auction_callback, this);

        private_nh.param("num_bids", num_bids_, 5000);
        private_nh.param("num_starts", num_starts_, 5000);
    }

    int num_bids_ = 5000;
    int num_starts_ = 5000;

    int start_hx_ct_ = 0;
    int bid_hx_ct_ = 0;
    int winner_id_ = -1;

    ros::Subscriber sub_result_;
    ros::Subscriber sub_start_;
    ros::Subscriber sub_bid_;

 public:
    void start_auction_callback(const RosStartAuction::ConstPtr &/*msg*/) {
        start_hx_ct_ += 1;
    }

    void bid_auction_callback(const RosBidAuction::ConstPtr &/*msg*/) {
        bid_hx_ct_ += 1;
    }

    void result_auction_callback(const RosBidAuction::ConstPtr &msg) {
        winner_id_ = msg->sender_id;
    }
};

TEST_F(AuctionTest, msg) {

    ros::Rate loop_rate(0.1);
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(bid_hx_ct_, num_bids_);
    EXPECT_EQ(start_hx_ct_, num_starts_);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "auction_test");
    ros::start();
    return RUN_ALL_TESTS();
}
