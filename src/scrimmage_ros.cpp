#include <scrimmage_ros/scrimmage_ros.h>

#include <iostream>
#include <memory>
#include <array>
#include <stdexcept>
#include <cstdio>

#include <XmlRpcValue.h>
#include <time.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

namespace sc = scrimmage;
namespace fs = boost::filesystem;
using std::cout;
using std::endl;

namespace scrimmage_ros {
scrimmage_ros::scrimmage_ros(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                             const std::string &node_name) :
    nh_(nh), private_nh_(private_nh), node_name_(node_name) {
}

bool scrimmage_ros::init(std::ostream &out) {
    if (not private_nh_.getParam("loop_rate_hz", loop_rate_hz_)) {
        out << node_name_ << ": missing ros param: loop_rate_hz." << endl;
    }

    std::string mission_file;
    if (not private_nh_.getParam("mission_file", mission_file)) {
        out << node_name_ << ": missing ros param: mission_file." << endl;
        return false;
    }

    std::string entity_tag;
    if (not private_nh_.getParam("entity_tag", entity_tag)) {
        out << node_name_ << ": missing ros param: entity_tag." << endl;
        return false;
    }

    std::string plugin_tags_str;
    if (not private_nh_.getParam("plugin_tags", plugin_tags_str)) {
        out << node_name_ << ": missing ros param: plugin_tags." << endl;
    }

    if (not private_nh_.getParam("entity_id", entity_id_)) {
        out << node_name_ << ": missing ros param: entity_id." << endl;
    }

    int max_contacts = 100;
    if (not private_nh_.getParam("max_contacts", max_contacts)) {
        out << node_name_ << ": missing ros param: max_contacts." << endl;
    }

    auto param_override_func = [&](std::map<std::string, std::string>& param_map) {
        for (auto &kv : param_map) {
            std::string resolved_param;
            if (private_nh_.searchParam(kv.first, resolved_param)) {
                XmlRpc::XmlRpcValue xmlrpc;
                if (private_nh_.getParam(resolved_param, xmlrpc)) {
                    if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeInvalid) {
                        out << node_name_ << ": Invalid XmlRpc param: " << resolved_param << endl;
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeBoolean) {
                        kv.second = std::to_string(bool(xmlrpc));
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeInt) {
                        kv.second = std::to_string(int(xmlrpc));
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeDouble) {
                        kv.second = std::to_string(double(xmlrpc));
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeString) {
                        kv.second = std::string(xmlrpc);
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeDateTime) {
                        struct tm time = tm(xmlrpc);
                        kv.second = std::to_string(timegm(&time));
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeBase64) {
                        out << node_name_ << ": Unsupported XmlRpc type (TypeBase64): " << resolved_param << endl;
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeArray) {
                        out << node_name_ << ": Unsupported XmlRpc type (TypeArray): " << resolved_param << endl;
                    } else if (xmlrpc.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct) {
                        out << node_name_ << ": Unsupported XmlRpc type (TypeSTruct): " << resolved_param << endl;
                    } else {
                        out << node_name_ << ": Can't parse XmlRpc param: " << resolved_param << endl;
                    }
                } else {
                    out << node_name_ << ": Failed to retrieve XMLRpc value for: " << resolved_param << endl;
                }
            }
        }
    };

    // Get the current ROS log directory
    ros_log_dir_ = scrimmage_ros::exec_command("roslaunch-logs");
    ros_log_dir_.erase(std::remove(ros_log_dir_.begin(), ros_log_dir_.end(), '\n'), ros_log_dir_.end());
    if (not fs::exists(fs::path(ros_log_dir_))) {
        out << node_name_ << ": ROS log directory doesn't exist: " << ros_log_dir_ << endl;
    }

    const bool create_entity =
        external_.create_entity(mission_file, entity_tag, plugin_tags_str,
                                entity_id_, max_contacts,
                                ros_log_dir_ + "/scrimmage",
                                param_override_func);
    if (create_entity) {
        external_.print_plugins(out);
    } else {
        out << node_name_ << ": failed to load plugins for " << entity_tag << endl;
    }
    return true;
}

bool scrimmage_ros::step(const double &t, std::ostream &out) {
    if (not external_.step(t)) {
        out << node_name_ << ": external step returned false." << endl;
        return false;
    }
    return true;
}

std::string scrimmage_ros::exec_command(const char* cmd) {
    std::array<char, 512> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 512, pipe.get()) != nullptr) {
            result += buffer.data();
        }
    }
    return result;
}

const double & scrimmage_ros::loop_rate_hz() {
    return loop_rate_hz_;
}

} // scrimmage_ros
