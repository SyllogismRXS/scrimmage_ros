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

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    XmlRpc::XmlRpcValue req = "/node";
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;

    if(not ros::master::execute("getSystemState", req, res, pay, true)) {
        std::cout << "Failed to call ros::master::execute getSystemState" << std::endl;
        return false;
    }
    // Response format is:
    //   [code,
    //    status,
    //    systemstate] i.e. res[2]
    // systemstate:
    //   [publishers,
    //    subscribers,
    //    services]    i.e. res[2][2]
    // services:
    //   [service, [service, [...]]] i.e. res[2][2][i]
    // service:
    //   [servicename, i.e. res[2][2][i][0]
    //    [serviceprovider, [serviceprovider [...]]]
    if (res.size() < 3 || res[2].size() < 3) {
        std::cout << "Invalid response from getSystemState" << std::endl;
        return false;
    }

    // Get the current namespace
    std::string this_namespace = ros::this_node::getNamespace(); // defined as '//<namespace.'
    if (this_namespace.size() < 2) {
      return false; // something is wrong if we don't see the // prefix
    }
    this_namespace = this_namespace.substr(2, std::string::npos); // save the name itself
    //std::cout << "dynamic_param_client::update_dynamic_param_servers: Current ROS namespace: " << this_namespace << std::endl;

    std::vector<std::string> updated_service_names_list;
    for(int i = 0; i < res[2][2].size(); i++) {
        // Get list of services
        std::string gh = res[2][2][i][0].toXml().c_str();
        //std::cout << "dynamic_param_client::update_dynamic_param_servers: service string: " << gh << std::endl;

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
            std::cout << "Ignoring badly formatted service name: " << gh << std::endl;
            continue;
        }

        std::size_t end_pos_of_first_delim = first_pos + start_delim_.length();
        std::string service_name = gh.substr(end_pos_of_first_delim,
                                             last_pos-end_pos_of_first_delim);

        // Determine if this service_name uses dynamic_reconfigure's
        // set_parameters service name
        std::size_t set_param_pos = service_name.find("/set_parameters");
        if (set_param_pos == std::string::npos) {
            continue; // not a service we care about
        }

        // Determine if this node is inside this current node's namespace
        std::string node_name = service_name.substr(0, set_param_pos);
        std::size_t found_this_namespace = node_name.find(this_namespace);
        if (found_this_namespace == std::string::npos) {
            continue; // not a service we care about
        }

        // Save the final found node that has the matching service
        updated_service_names_list.push_back(node_name);
        //std::cout << "dynamic_param_client::update_dynamic_param_servers: Found service: " << node_name << std::endl;
    }
    // Populate the updated client list
    std::unordered_map<std::string, std::shared_ptr<dynamic_reconfigure::Client<scrimmage_rosConfig>>> updated_services_list;
    for(const std::string& node_name : updated_service_names_list) {
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
            for( const scrimmage_rosConfig& config :  current_config_list ) {
                new_client->setConfiguration(config);
            }
        }
    }
    services_.swap(updated_services_list); //save the result

    return true;
}

bool dynamic_param_client::send_config(const scrimmage_rosConfig &config) {

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto send_config = [&](auto &s) {
      //printf("dynamic_param_client::send_config: service: %s name: %s type: %d value: %s\n", s.first.c_str(), config.param_name.c_str(), (int) config.param_type, config.param_value.c_str());
      return s.second->setConfiguration(config);
    };

    return std::all_of(services_.begin(), services_.end(), send_config);
}
} // namespace scrimmage_ros
