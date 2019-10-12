#ifndef SCRIMMAGE_ROS_DYNAMIC_PARAM_CLIENT_H_
#define SCRIMMAGE_ROS_DYNAMIC_PARAM_CLIENT_H_

#include "ros/ros.h"
#include <scrimmage_ros/scrimmage_rosConfig.h>
#include <scrimmage_ros/ScrimmageNodeAdvertise.h>

// There's a warning being generated in the ROS header because of out-of-order
// initialization. We don't want to see that particular warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include <dynamic_reconfigure/client.h>
#pragma GCC diagnostic pop

#include <string>
#include <unordered_map>
#include <mutex>

namespace scrimmage_ros {
class dynamic_param_client {
 public:
    explicit dynamic_param_client(const std::string &name = "dyn_param");
    // init()
    //   Create a service (server side) with message type "string"
    //   that contains the scrimmage_ros node's name. Adds name to
    //   std::set which keeps track of acceptable nodes for later
    //   dynamic reconfigure mesasges. Function keeps track of
    //   include_nodes_ variable.
    void init(ros::NodeHandle nh);
    bool advertise_node_name(scrimmage_ros::ScrimmageNodeAdvertise::Request &req,
                             scrimmage_ros::ScrimmageNodeAdvertise::Response &res);
    // update_dynamic_param_servers
    //   Updates the internal list of all dynamic reconfigure ros client services.
    //   For any new services that are found, the given function will be run to
    //   generate a current list of config values, which will then all be passed
    //   to the newly added client service. Prior known services are not modified.
    //   Thread Safe.
    bool update_dynamic_param_servers(
           std::function<void(std::vector<scrimmage_rosConfig>&)> generate_current_config_list = std::function<void(std::vector<scrimmage_rosConfig>&)>());
    // send_config
    //   Sends the given config value to all currently known dynamic reconfigure ros client services.
    //   Thread Safe.
    bool send_config(const scrimmage_rosConfig &config);
    // services
    //   Get the currently known dynamic reconfigure ros client services list.
    //   NOT Thread Safe.
    const std::unordered_map<std::string, std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>> &services() {
        return services_;
    }
    // exclude_nodes
    //   Returns a modifiable set of strings, identifying the nodes (names) that will not
    //   be added, meaning they are excluded, during the update_dynamic_param_servers call.
    //   NOT Thread Safe.
    std::set<std::string> &exclude_nodes() { return exclude_nodes_; }
    // include_nodes
    //   Returns a modifiable set of strings, identifying the nodes
    //   (names) that will be added during the
    //   update_dynamic_param_server's call.
    std::set<std::string> &include_nodes() { return include_nodes_; }
    // ROS service server handle
    ros::ServiceServer scNodeAdvertiseService_;

 protected:
    //global mutex on data
    std::recursive_mutex mutex_;

    std::string name_;
    std::unordered_map<std::string, std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>> services_;
    const std::string start_delim_ = "<value>";
    const std::string end_delim_ = "</value>";
    std::set<std::string> exclude_nodes_;
    std::set<std::string> include_nodes_;
};
} // namespace scrimmage_ros
#endif // SCRIMMAGE_ROS_DYNAMIC_PARAM_CLIENT_H_
