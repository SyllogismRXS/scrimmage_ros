#ifndef SCRIMMAGE_ROS_ENTITY_H_
#define SCRIMMAGE_ROS_ENTITY_H_

#include <scrimmage/entity/External.h>

namespace sc = scrimmage;

namespace scrimmage_ros {
class Entity {
public:
    Entity();
    bool init(int argc, char *argv[], std::string node_name);
    bool wait_until_ready();
    bool run();

    // bool init_scrimmage();
    // bool update_contacts();
    // bool update_own_state();
    // bool add_contact(scrimmage::ID id);
    //
    // void contact_list_cb(const scrimmage_ros::ContactArray::ConstPtr& msg);
    // void amcl_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

protected:
    std::string node_name_;
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::Rate> loop_rate_;

    sc::External external_;

private:
};
} // namespace scrimmage_ros
#endif // SCRIMMAGE_ROS_ENTITY_H_
