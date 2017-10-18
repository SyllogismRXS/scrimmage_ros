#include <iostream>
#include <map>

#include <boost/regex.hpp>
#include <Eigen/Dense>

#include <scrimmage/common/RTree.h>
#include <scrimmage/math/State.h>

#include "ros/ros.h"
#include <tf/tf.h>

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <scrimmage_ros/scrimmage_autonomy.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

ScrimmageAutonomy::ScrimmageAutonomy() : valid_own_pose_(false),
                                         valid_contact_list_msg_(false),
                                         initScrimmagePluginStatus_(false)
{
    //external_.set_max_entities(100);
}

bool ScrimmageAutonomy::init(int argc, char *argv[], std::string node_name){

    node_name_ = node_name;
    ros::init(argc, argv, node_name_);
    nh_ = std::make_shared<ros::NodeHandle>();

    ros::NodeHandle private_nh("~");
    double loop_rate;
    private_nh.param("loop_rate", loop_rate, 10.0);
    loop_rate_ = std::make_shared<ros::Rate>(loop_rate);

    setup_publishers();
    setup_subscribers();

    ROS_INFO("ScrimmageAutonomy Initialization Complete");

    return true;
}

bool ScrimmageAutonomy::setup_publishers()
{
    goal_pub_ = nh_->advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
    herding_goal_pub_ = nh_->advertise<visualization_msgs::Marker>("herding_goal", 1);
    return true;
}

void ScrimmageAutonomy::contact_list_cb(const scrimmage_ros::ContactArray::ConstPtr& msg)
{
    contact_list_msg_ = *msg;
    valid_contact_list_msg_ = true;
}

void ScrimmageAutonomy::amcl_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    own_pose_ = *msg;
    valid_own_pose_ = true;
}

bool ScrimmageAutonomy::setup_subscribers()
{
    contact_list_sub_ = nh_->subscribe("/contact_list", 10, &ScrimmageAutonomy::contact_list_cb, this);
    amcl_pose_sub_ = nh_->subscribe("amcl_pose", 1, &ScrimmageAutonomy::amcl_pose_cb, this);
    return true;
}

bool ScrimmageAutonomy::init_scrimmage()
{
    ros::NodeHandle private_nh("~");

    std::string autonomy_plugin;
    if (!private_nh.getParam("autonomy_plugin", autonomy_plugin)) {
        return false;
    }

    std::string control_mode_str = "heading_speed";
    private_nh.param("control_mode", control_mode_str, std::string("heading_speed"));
    if (control_mode_str == "waypoint") {
        control_mode_ = WAYPOINT;
    } else if (control_mode_str == "heading_speed") {
        control_mode_ = HEADING_SPEED;
    } else if (control_mode_str == "velocity") {
        control_mode_ = VELOCITY;
    } else {
        control_mode_ = HEADING_SPEED;
    }

    int robot_id, team_id;
    private_nh.param("robot_id", robot_id, 1);
    private_nh.param("team_id", team_id, 1);
    private_nh.param("name_prefix", name_prefix, std::string("robot"));
    private_nh.param("robot_id", robot_id, 1);
    private_nh.param("wp_forw_time", wp_forw_time_, 1.0);
    private_nh.param("visualize_herding_goal", visualize_herding_goal_, false);

    id_ = sc::ID(robot_id, -1, team_id);
    cout << "Own ID: " << id_.id() << endl;
    cout << "Team ID: " << id_.team_id() << endl;

    // Initialize scrimmage plugin:
    std::map<std::string, std::string> info;
    info["x"] = std::to_string(own_pose_.pose.pose.position.x);
    info["y"] = std::to_string(own_pose_.pose.pose.position.y);
    info["z"] = std::to_string(own_pose_.pose.pose.position.z);
    info["latitude"] = std::to_string(35.721025);
    info["longitude"] = std::to_string(-120.767925);
    info["heading"] = std::to_string(tf::getYaw(own_pose_.pose.pose.orientation));
    info["autonomy0"] = autonomy_plugin;
    //info["controller0"] = "JSBSimModelControllerHeadingPID";

    if (external_.create_entity(100, id_, info, "~/.scrimmage/logs/ros")) {
        //contacts_ = std::make_shared<sc::ContactMap>();
        update_contacts();
        external_.update_contacts = std::bind(&ScrimmageAutonomy::update_contacts, this);
        autonomy_ = external_.entity()->autonomies().front();
        contacts_ = external_.entity()->contacts();
        autonomy_->set_contacts(contacts_);
        external_.entity()->id() = id_; // TODO: needed?
    } else {
        cout << "Failed to create SCRIMMAGE entity" << endl;
        return false;
    }
    cout << "SCRIMMAGE Initialization Complete." << endl;
    return true;
}

bool ScrimmageAutonomy::add_contact(sc::ID id)
{
    sc::Contact c;
    c.set_id(id);
    c.set_type(sc::Contact::Type::UNKNOWN);
    (*contacts_)[id.id()] = c;

    cout << "Added contact ID: " << c.id().id()
         << ", team: " << c.id().team_id() << endl;
    return true;
}

bool ScrimmageAutonomy::update_own_state()
{
    // Set position
    external_.entity()->state()->pos() << own_pose_.pose.pose.position.x,
        own_pose_.pose.pose.position.y, own_pose_.pose.pose.position.z;

    //external_.entity()->state()->vel() << own_pose_.pose.pose.orienation.x,
    //    own_pose_.pose.pose.orientation., own_pose_.twist.twist.linear.z;

    // Set orientation
    sc::Quaternion quat(own_pose_.pose.pose.orientation.w,
                        own_pose_.pose.pose.orientation.x,
                        own_pose_.pose.pose.orientation.y,
                        own_pose_.pose.pose.orientation.z);
    double roll = quat.roll();
    double pitch = quat.pitch();
    double yaw = quat.yaw();
    if (std::isnan(roll)) roll = 0;
    if (std::isnan(pitch)) pitch = 0;
    if (std::isnan(yaw)) yaw = 0;

    external_.entity()->state()->quat().set(roll, pitch, yaw);
    return true;
}

bool ScrimmageAutonomy::update_contacts()
{
    if (autonomy_ == nullptr || contacts_ == nullptr) {
        return false;
    }

    //update_own_state(); // update through contacts for now

    sc::RTreePtr &rtree = autonomy_->rtree();
    rtree->init(contact_list_msg_.contact_list.size());
    rtree->clear();

    for (scrimmage_ros::Contact pose : contact_list_msg_.contact_list) {
        // // TODO: Need to create message with ID in header, parse for now
        // std::string str = std::string(pose.header.frame_id);
        // std::string regex_str = name_prefix + std::string("(\\d+)\\/map");

        // boost::regex re(regex_str);
        // boost::sregex_iterator rit(str.begin(), str.end(), re);
        // cnt_id = std::stoi(rit->str(1)); // first match is entire string,
        //                                  // second match is ID

        int cnt_id = pose.robot_id.id;

        // Find the contact based on its ID. If it doesn't exist, create it.
        auto it_contact = contacts_->find(cnt_id);
        if (it_contact == contacts_->end()) {
            // TODO: team_id should be in message
            // int team_id = cnt_id;
            int team_id = pose.robot_id.team_id;
            add_contact(sc::ID(cnt_id, -1, team_id));
            it_contact = contacts_->find(cnt_id);
        }

        // Update contact's state from amcl_pose message
        sc::Contact &contact = it_contact->second;
        sc::StatePtr &state = contact.state();
        state->pos() << pose.pose.pose.position.x, pose.pose.pose.position.y,
            pose.pose.pose.position.z;
        state->vel() << pose.twist.twist.linear.x, pose.twist.twist.linear.y,
            pose.twist.twist.linear.z;

        // Set orientation
        sc::Quaternion quat(pose.pose.pose.orientation.w,
                            pose.pose.pose.orientation.x,
                            pose.pose.pose.orientation.y,
                            pose.pose.pose.orientation.z);
        double roll = quat.roll();
        double pitch = quat.pitch();
        double yaw = quat.yaw();
        if (std::isnan(roll)) roll = 0;
        if (std::isnan(pitch)) pitch = 0;
        if (std::isnan(yaw)) yaw = 0;
        state->quat().set(roll, pitch, yaw);

        // If this is our own ID, update our own state
        if (cnt_id == id_.id()) {
            external_.entity()->state() = contact.state();
            autonomy_->state() = contact.state();
        }

        rtree->add(state->pos(), contact.id());
    }
    return true;
}

bool ScrimmageAutonomy::wait_until_ready()
{
    while (ros::ok()) {
        if (valid_own_pose_ && valid_contact_list_msg_) {
            break;
        }
        ros::spinOnce();
        loop_rate_->sleep();
    }
    cout << "ScrimmageAutonomy node ready!" << endl;
    return true;
}

visualization_msgs::Marker ScrimmageAutonomy::setup_herding_marker(Eigen::Vector3d goal)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "herding goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal(0);
    marker.pose.position.y = goal(1);
    marker.pose.position.z = goal(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.r = 0;
    marker.color.g = 100;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(5);

    return marker;
}

bool ScrimmageAutonomy::run()
{
    wait_until_ready();
    if (!init_scrimmage()) {
        cout << "Failed to initialize SCRIMMAGE plugin" << endl;
        return false;
    }

    move_base_msgs::MoveBaseActionGoal goal;
    goal.header.stamp = ros::Time::now();
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.pose.position.x = 0;
    goal.goal.target_pose.pose.position.y = 0;
    goal.goal.target_pose.pose.position.z = 0;
    goal.goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    ros::Time prev_time = ros::Time::now();
    ros::Duration one_sec(1.0);

    // TODO: read goal value from Herder.xml mission file
    Eigen::Vector3d herding_goal(-7.0, -4.5, 0.0);
    visualization_msgs::Marker herding_marker = setup_herding_marker(herding_goal);

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        double t = now.toSec();

        // Update the autonomy plugin's contacts, step the autonomy, and get
        // the desired state
        external_.step(t);
        sc::StatePtr desired_state = autonomy_->desired_state();

        Eigen::Vector3d wp(0,0,0);
        double wp_heading = 0;
        if(control_mode_ == VELOCITY) {
            wp = autonomy_->state()->pos() + desired_state->vel() * wp_forw_time_;
            if (desired_state->vel().norm() >= 0.001) {
                wp_heading = atan2(desired_state->vel()(1), desired_state->vel()(0));
            } else {
                wp_heading = autonomy_->state()->quat().yaw();
            }

        } else if (control_mode_ == HEADING_SPEED) {
            double speed = desired_state->vel()(0);
            wp_heading = desired_state->quat().yaw();

            Eigen::Rotation2D<double> rot(wp_heading);

            Eigen::Vector2d wp2 = autonomy_->state()->pos().head<2>() +
                rot.toRotationMatrix() * Eigen::Vector2d::UnitX() * speed * wp_forw_time_;
            wp(0) = wp2(0);
            wp(1) = wp2(1);
            wp(2) = autonomy_->state()->pos()(2);

        } else if (control_mode_ == WAYPOINT) {
            wp = desired_state->pos();
            wp_heading = desired_state->quat().yaw();
        }

        goal.goal.target_pose.pose.position.x = wp(0);
        goal.goal.target_pose.pose.position.y = wp(1);
        goal.goal.target_pose.pose.position.z = wp(2);

        goal.goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(wp_heading);

        if ((now - prev_time) > one_sec) {
            prev_time = now;
            goal.header.stamp = ros::Time::now();
            goal_pub_.publish(goal);
        }

        if (visualize_herding_goal_)
            herding_goal_pub_.publish(herding_marker);

        ros::spinOnce();
        loop_rate_->sleep();
    }
    return true;
}

int main(int argc, char **argv)
{
    ScrimmageAutonomy sc_autonomy;
    sc_autonomy.init(argc, argv, "scrimmage_autonomy");
    if (!sc_autonomy.run()) {
        return -2;
    }
    return 0;
}
