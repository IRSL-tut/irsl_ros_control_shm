#pragma once

#include "RobotHWShm.h"
#include "irsl_dynamixel_hardware_shm/DynamixelShmLib.h"
#include <memory>

namespace irsl_ros_control_shm {

class RobotDxHWShm : public RobotHWShm
{
public:
    RobotDxHWShm() = default;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
    hardware_interface::CallbackReturn initializeBackend() override;

public:
    irsl_dynamixel::DynamixelShmPtr dx_shm_ptr;
};

typedef std::shared_ptr<RobotDxHWShm> RobotDxHWShmPtr;

}  // namespace irsl_ros_control_shm
