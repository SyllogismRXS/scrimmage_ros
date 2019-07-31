#include <scrimmage_ros/dynamic_param_client.h>

#include <ros/ros.h>

#include <string>
#include <algorithm>
#include <iostream>

namespace scrimmage_ros {

dynamic_param_client::dynamic_param_client(const std::string &name)
    : name_(name) {}

bool dynamic_param_client::update_dynamic_param_servers(
       std::function<void(std::vector<scrimmage_rosConfig>&)> generate_current_config_list) {
    XmlRpc::XmlRpcValue req = "/node";
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;

    if(not ros::master::execute("getSystemState", req, res, pay, true)) {
        std::cout << "Failed to call ros::master::execute getSystemState" << std::endl;
        return false;
    }
    if (res.size() < 3 || res[2].size() < 3) {
        std::cout << "Invalid response from getSystemState" << std::endl;
        return false;
    }

    std::vector<std::string> updated_service_names_list;
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
            std::cout << "Ignore non-scrimmage service: " << gh << std::endl;
            continue;
        }

        std::size_t end_pos_of_first_delim = first_pos + start_delim_.length();
        std::string service_name = gh.substr(end_pos_of_first_delim,
                                             last_pos-end_pos_of_first_delim);

        // Determine if this service_name uses dynamic_reconfigure's
        // set_parameters service name
        std::size_t set_param_pos = service_name.find("/set_parameters");
        if (set_param_pos != std::string::npos) {
            std::string node_name = service_name.substr(0, set_param_pos);
            updated_service_names_list.push_back(node_name);
        }
    }
    // Populate the updated client list
    std::unordered_map<std::string, std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>> updated_services_list;
    auto move_or_create_client = [&](const std::string& node_name) -> void {
        // check if this node is already in the list
        std::unordered_map<std::string,
          std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>
          >::iterator found_element;
        found_element = services_.find(node_name);
        if (found_element != services_.end()) {
            // this one already exists, just move it to the new list
            updated_services_list[node_name] = found_element->second; //shared_ptr copy
        } else {
            // this one is new, create it and initialize it
            ros::NodeHandle nh(node_name);
            std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>> new_client =
                std::make_shared<dynamic_reconfigure::Client<scrimmage_rosConfig>>(name_, nh);
            updated_services_list[node_name] = new_client;
            // initialize it
            //   get the current configs from the caller
            std::vector<scrimmage_rosConfig> current_config_list;
            if(generate_current_config_list) generate_current_config_list(current_config_list);
            //   send each one to the new client
            std::for_each(current_config_list.cbegin(), current_config_list.cend(),
                [&new_client](const scrimmage_rosConfig& config) -> void {new_client->setConfiguration(config);});
        }
    };
    std::for_each(updated_service_names_list.cbegin(), updated_service_names_list.cend(), move_or_create_client);
    services_.swap(updated_services_list); //save the result

    return true;
}

bool dynamic_param_client::send_config(const scrimmage_rosConfig &config) {
    auto send_config = [&](auto &s) { 
      std::cout << "dynamic_param_client::send_config: node_name: " << s.first.c_str() << " param_name: " << config.param_name << " param_value: " << config.param_value << std::endl;
      return s.second->setConfiguration(config); };
    return std::all_of(services_.begin(), services_.end(), send_config);
}
} // namespace scrimmage_ros
