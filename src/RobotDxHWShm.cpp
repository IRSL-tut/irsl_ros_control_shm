#include "RobotDxHWShm.h"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using namespace std;

namespace irsl_ros_control_shm {

namespace {

rclcpp::Logger logger()
{
    return rclcpp::get_logger("irsl_ros_control_shm.dx");
}

}  // namespace

hardware_interface::CallbackReturn RobotDxHWShm::initializeBackend()
{
    const auto config_it = info_.hardware_parameters.find("config");
    if (config_it == info_.hardware_parameters.end() || config_it->second.empty()) {
        RCLCPP_ERROR(logger(), "Missing required hardware parameter 'config' for RobotDxHWShm");
        return hardware_interface::CallbackReturn::ERROR;
    }

    uint64_t hash = 8888;
    uint32_t shm_key = 8888;
    const auto hash_it = info_.hardware_parameters.find("hash");
    if (hash_it != info_.hardware_parameters.end() && !hash_it->second.empty()) {
        hash = std::stoull(hash_it->second);
    }
    const auto shm_key_it = info_.hardware_parameters.find("shm_key");
    if (shm_key_it != info_.hardware_parameters.end() && !shm_key_it->second.empty()) {
        shm_key = static_cast<uint32_t>(std::stoul(shm_key_it->second));
    }

    dx_shm_ptr = std::make_shared<irsl_dynamixel::DynamixelShm>();
    dx_shm_ptr->initialize(config_it->second, hash, shm_key);
    dx_shm_ptr->readDx();
    dx_shm_ptr->initializeCommand();
    setShmManager(dx_shm_ptr->shm_manager().get());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotDxHWShm::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    dx_shm_ptr->writeDx();
    return RobotHWShm::read(time, period);
}

hardware_interface::return_type RobotDxHWShm::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    dx_shm_ptr->readDx();
    return RobotHWShm::write(time, period);
}

}  // namespace irsl_ros_control_shm

PLUGINLIB_EXPORT_CLASS(irsl_ros_control_shm::RobotDxHWShm, hardware_interface::SystemInterface)
