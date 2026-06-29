#pragma once

#include <hardware_interface/system_interface.hpp>

#include <memory>
#include <string>
#include <vector>

#include "joint_info.h"

namespace irsl_shm_controller {
class ShmManager;
}

namespace irsl_ros_control_shm {

class RobotHWShm : public hardware_interface::SystemInterface
{
public:
    RobotHWShm();
    ~RobotHWShm() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    void setShmManager(irsl_shm_controller::ShmManager *ptr);
    hardware_interface::CallbackReturn initializeJoints(const std::vector<joint_info> &joints);
    virtual hardware_interface::CallbackReturn initializeBackend();

private:
    class Impl;
    std::unique_ptr<Impl> impl;
};

typedef std::shared_ptr<RobotHWShm> RobotHWShmPtr;

}  // namespace irsl_ros_control_shm
