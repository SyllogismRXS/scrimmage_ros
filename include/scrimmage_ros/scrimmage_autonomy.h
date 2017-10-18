#ifndef SCRIMMAGE_ROS_SCRIMMAGE_AUTONOMY_H_
#define SCRIMMAGE_ROS_SCRIMMAGE_AUTONOMY_H_

#include <scrimmage/entity/External.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/motion/Controller.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <scrimmage_ros/ContactArray.h>
#include <memory>
#include <visualization_msgs/Marker.h>

class ScrimmageAutonomy {
public:

    typedef enum ControlMode {
        WAYPOINT = 0,
        HEADING_SPEED,
        VELOCITY
    } ControlMode_t;

    ScrimmageAutonomy();
    bool init(int argc, char *argv[], std::string node_name);
    bool setup_publishers();
    bool setup_subscribers();
    bool wait_until_ready();
    bool run();

    bool init_scrimmage();
    bool update_contacts();
    bool update_own_state();
    bool add_contact(scrimmage::ID id);

    void contact_list_cb(const scrimmage_ros::ContactArray::ConstPtr& msg);
    void amcl_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

protected:
    std::string node_name_;
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::Rate> loop_rate_;

    ros::Publisher goal_pub_;
    ros::Subscriber contact_list_sub_;
    ros::Subscriber amcl_pose_sub_;

    bool valid_own_pose_;
    geometry_msgs::PoseWithCovarianceStamped own_pose_;

    bool valid_contact_list_msg_;
    scrimmage_ros::ContactArray contact_list_msg_;

    scrimmage::External external_;
    scrimmage::AutonomyPtr autonomy_;
    scrimmage::ContactMapPtr contacts_;
    bool initScrimmagePluginStatus_;

    scrimmage::ID id_;

    std::string name_prefix;
    ControlMode_t control_mode_;
    double wp_forw_time_;

private:
};

#endif
