#include <scrimmage_ros/dynamic_param_client.h>

#include <ros/ros.h>

#include <string>
#include <algorithm>
#include <iostream>

namespace scrimmage_ros {

dynamic_param_client::dynamic_param_client(const std::string &name)
    : name_(name) {}

bool dynamic_param_client::update_dynamic_param_servers() {
    services_.clear();

    XmlRpc::XmlRpcValue req = "/node";
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;

    ros::master::execute("getSystemState", req, res, pay, true);
    for(int i = 0; i < res[2][2].size(); i++) {
        // Get list of services
        std::string gh = res[2][2][i][0].toXml().c_str();

        // Returns true if the service string (gh) contains "str"
        auto exclude = [&] (const std::string &str) {
                           return gh.find(str) != std::string::npos;
                       };
        // Does this service belong to a node that we are excluding?
        if (std::any_of(exclude_nodes_.begin(), exclude_nodes_.end(),
                        exclude)) {
            continue;
        }

        // Get the service name between the <value> </value> tags
        std::size_t first_pos = gh.find(start_delim_);
        std::size_t last_pos = gh.find_last_of(end_delim_);
        if (first_pos == std::string::npos || last_pos == std::string::npos) {
            std::cout << "Invalid service: " << gh << std::endl;
            return false;
        }

        std::size_t end_pos_of_first_delim = first_pos + start_delim_.length();
        std::string service_name = gh.substr(end_pos_of_first_delim,
                                             last_pos-end_pos_of_first_delim);

        // Determine if this service_name uses dynamic_reconfigure's
        // set_parameters service name
        std::size_t set_param_pos = service_name.find("/set_parameters");
        if (set_param_pos != std::string::npos) {
            std::string node_name = service_name.substr(0, set_param_pos);
            ros::NodeHandle nh(node_name);
            services_[node_name] =
                std::make_shared<dynamic_reconfigure::Client<scrimmage_rosConfig>>(name_, nh);
        }
    }
    return true;
}

bool dynamic_param_client::send_config(const scrimmage_rosConfig &config) {
    auto send_config = [&](auto &s) { return s.second->setConfiguration(config); };
    return std::all_of(services_.begin(), services_.end(), send_config);
}
} // namespace scrimmage_ros
