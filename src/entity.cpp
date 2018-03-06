#include <iostream>
#include <map>

#include <boost/regex.hpp>
#include <Eigen/Dense>

#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/SubscriberBase.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/PluginManager.h>

#include "ros/ros.h"
#include <tf/tf.h>

#include <boost/function.hpp>

#include <std_msgs/Float32.h>

// #include <move_base_msgs/MoveBaseActionGoal.h>
#include <scrimmage_ros/entity.h>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

namespace scrimmage_ros {
Entity::Entity() {
}

bool Entity::init(int argc, char *argv[], std::string node_name){
    node_name_ = node_name;
    ros::init(argc, argv, node_name_);
    nh_ = std::make_shared<ros::NodeHandle>();

    ros::NodeHandle private_nh("~");
    double loop_rate;
    private_nh.param("loop_rate", loop_rate, 10.0);
    loop_rate_ = std::make_shared<ros::Rate>(loop_rate);

    std::string mission_file;
    private_nh.param("mission_file", mission_file, std::string(""));

    // Get the entity ID
    int entity_id;
    private_nh.param("entity_id", entity_id, 1);

    std::string entity_name;
    private_nh.param("entity_name", entity_name, std::string("UNDEFINED"));

    int max_contacts;
    private_nh.param("max_contacts", max_contacts, 100);

    external_.create_entity(mission_file, max_contacts, entity_id, entity_name);

    // std::string network_name = "GlobalNetwork";
    // std::string sc_topic_name = "ANumber";
    //
    // std::string ros_topic_name = "Number";
    //
    // msg_plugin_ = std::make_shared<sc::Plugin>();
    // msg_plugin_->set_pubsub(pubsub_);
    //
    // sc::PublisherPtr pub = msg_plugin_->advertise(network_name, sc_topic_name);
    // pubs_to_scrimmage_.push_back(pub);
    //
    // boost::function<void(const boost::shared_ptr<std_msgs::Float32 const>&)> callback =
    //     [&] (const boost::shared_ptr<std_msgs::Float32 const>&msg) {
    //     cout << "Got ROS data: " << *msg << endl;
    //
    // };
    //
    // ros_subs_.push_back(nh_->subscribe(ros_topic_name, 1, callback));

    ROS_INFO("Entity Initialization Complete");

    return true;
}

// void Entity::contact_list_cb(const scrimmage_ros::ContactArray::ConstPtr& msg)
// {
//     contact_list_msg_ = *msg;
//     valid_contact_list_msg_ = true;
// }
//
// void Entity::amcl_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// {
//     own_pose_ = *msg;
//     valid_own_pose_ = true;
// }

// bool Entity::setup_subscribers()
// {
//     // contact_list_sub_ = nh_->subscribe("/contact_list", 10, &Entity::contact_list_cb, this);
//     // amcl_pose_sub_ = nh_->subscribe("amcl_pose", 1, &Entity::amcl_pose_cb, this);
//     return true;
// }

// bool Entity::add_contact(sc::ID id)
// {
//     sc::Contact c;
//     c.set_id(id);
//     c.set_type(sc::Contact::Type::UNKNOWN);
//     (*contacts_)[id.id()] = c;
//
//     cout << "Added contact ID: " << c.id().id()
//          << ", team: " << c.id().team_id() << endl;
//     return true;
// }
//
// bool Entity::update_own_state()
// {
//     // Set position
//     external_.entity()->state()->pos() << own_pose_.pose.pose.position.x,
//         own_pose_.pose.pose.position.y, own_pose_.pose.pose.position.z;
//
//     //external_.entity()->state()->vel() << own_pose_.pose.pose.orienation.x,
//     //    own_pose_.pose.pose.orientation., own_pose_.twist.twist.linear.z;
//
//     // Set orientation
//     sc::Quaternion quat(own_pose_.pose.pose.orientation.w,
//                         own_pose_.pose.pose.orientation.x,
//                         own_pose_.pose.pose.orientation.y,
//                         own_pose_.pose.pose.orientation.z);
//     double roll = quat.roll();
//     double pitch = quat.pitch();
//     double yaw = quat.yaw();
//     if (std::isnan(roll)) roll = 0;
//     if (std::isnan(pitch)) pitch = 0;
//     if (std::isnan(yaw)) yaw = 0;
//
//     external_.entity()->state()->quat().set(roll, pitch, yaw);
//     return true;
// }
//
// bool Entity::update_contacts()
// {
//     if (autonomy_ == nullptr || contacts_ == nullptr) {
//         return false;
//     }
//
//     sc::RTreePtr &rtree = autonomy_->rtree();
//     rtree->init(contact_list_msg_.contact_list.size());
//     rtree->clear();
//
//     for (scrimmage_ros::Contact pose : contact_list_msg_.contact_list) {
//         // // TODO: Need to create message with ID in header, parse for now
//         // std::string str = std::string(pose.header.frame_id);
//         // std::string regex_str = name_prefix + std::string("(\\d+)\\/map");
//
//         // boost::regex re(regex_str);
//         // boost::sregex_iterator rit(str.begin(), str.end(), re);
//         // cnt_id = std::stoi(rit->str(1)); // first match is entire string,
//         //                                  // second match is ID
//
//         int cnt_id = pose.robot_id.id;
//
//         // Find the contact based on its ID. If it doesn't exist, create it.
//         auto it_contact = contacts_->find(cnt_id);
//         if (it_contact == contacts_->end()) {
//             // TODO: team_id should be in message
//             // int team_id = cnt_id;
//             int team_id = pose.robot_id.team_id;
//             add_contact(sc::ID(cnt_id, -1, team_id));
//             it_contact = contacts_->find(cnt_id);
//         }
//
//         // Update contact's state from amcl_pose message
//         sc::Contact &contact = it_contact->second;
//         sc::StatePtr &state = contact.state();
//         state->pos() << pose.pose.pose.position.x, pose.pose.pose.position.y,
//             pose.pose.pose.position.z;
//         state->vel() << pose.twist.twist.linear.x, pose.twist.twist.linear.y,
//             pose.twist.twist.linear.z;
//
//         // Set orientation
//         sc::Quaternion quat(pose.pose.pose.orientation.w,
//                             pose.pose.pose.orientation.x,
//                             pose.pose.pose.orientation.y,
//                             pose.pose.pose.orientation.z);
//         double roll = quat.roll();
//         double pitch = quat.pitch();
//         double yaw = quat.yaw();
//         if (std::isnan(roll)) roll = 0;
//         if (std::isnan(pitch)) pitch = 0;
//         if (std::isnan(yaw)) yaw = 0;
//         state->quat().set(roll, pitch, yaw);
//
//         // If this is our own ID, update our own state
//         if (cnt_id == id_.id()) {
//             external_.entity()->state() = contact.state();
//             autonomy_->state() = contact.state();
//         }
//
//         rtree->add(state->pos(), contact.id());
//     }
//     return true;
// }

bool Entity::wait_until_ready()
{
    // while (ros::ok()) {
    //     if (valid_own_pose_ && valid_contact_list_msg_) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     loop_rate_->sleep();
    // }
    return true;
}

bool Entity::run()
{
    wait_until_ready();

    // ros::Time prev_time = ros::Time::now();

    while (ros::ok()) {
        external_.step(ros::Time::now().toSec());

        // // Run the autonomies
        // for (sc::AutonomyPtr &a : entity_->autonomies()) {
        //     // Execute callbacks for received messages before calling
        //     // step_autonomy
        //     for (sc::SubscriberBasePtr &sub : a->subs()) {
        //         for (auto msg : sub->msgs<sc::MessageBase>(true)) {
        //             sub->accept(msg);
        //         }
        //     }
        //     if (!a->step_autonomy(time_->t(), time_->dt())) {
        //         cout << "Failed to step_autonomy" << endl;
        //     }
        // }
        //
        // // Run the controllers
        // for (sc::ControllerPtr &ctrl : entity_->controllers()) {
        //     // Execute callbacks for received messages before calling
        //     // controllers
        //     for (sc::SubscriberBasePtr &sub : ctrl->subs()) {
        //         for (auto msg : sub->msgs<sc::MessageBase>(true)) {
        //             sub->accept(msg);
        //         }
        //     }
        //     if (!ctrl->step(time_->t(), time_->dt())) {
        //         cout << "Failed to step controller" << endl;
        //     }
        // }
        //
        // prev_time_ = t_now;

        loop_rate_->sleep();
        ros::spinOnce();
    }
    return true;
}
} // namespace scrimmage_ros

int main(int argc, char **argv)
{
    scrimmage_ros::Entity entity;
    if (!entity.init(argc, argv, "scrimmage_entity")) {
        cout << "Failed to initialize entity" << endl;
        return -1;
    }
    if (!entity.run()) {
        return -2;
    }
    return 0;
}
