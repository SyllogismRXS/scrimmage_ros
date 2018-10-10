#ifndef SCRIMMAGE_ROS_DYNAMIC_PARAM_CLIENT_H_
#define SCRIMMAGE_ROS_DYNAMIC_PARAM_CLIENT_H_

#include "ros/ros.h"
#include <scrimmage_ros/scrimmage_rosConfig.h>
#include <dynamic_reconfigure/client.h>

#include <string>
#include <unordered_map>

namespace scrimmage_ros {
class dynamic_param_client {
 public:
    explicit dynamic_param_client(const std::string &name = "dyn_param");
    bool update_dynamic_param_servers();
    bool send_config(const scrimmage_rosConfig &config);
    const std::unordered_map<std::string, std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>> &services() {
        return services_;
    }
    std::set<std::string> &exclude_nodes() { return exclude_nodes_; }

 protected:
    std::string name_;
    std::unordered_map<std::string, std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>> services_;
    const std::string start_delim_ = "<value>";
    const std::string end_delim_ = "</value>";
    std::set<std::string> exclude_nodes_;
};
} // namespace scrimmage_ros
#endif // SCRIMMAGE_ROS_DYNAMIC_PARAM_CLIENT_H_
