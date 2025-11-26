#pragma once

#include "RobotHWShm.h"
#include "irsl_dynamixel_hardware_shm/DynamixelShmLib.h"
#include <memory>

namespace hardware_interface {

class RobotDxHWShm : public RobotHWShm
{
public:
    RobotDxHWShm() {};

    //virtual bool initSim(const ros::NodeHandle& nh, cnoid::ControllerIO* args) final;
    virtual bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &/*robot_hw_nh*/) {
        std::cerr << "initialize: RobotDxHWShm" << std::endl;
        return true;
    }
    virtual void read(const ros::Time& time, const ros::Duration& period) override;
    virtual void write(const ros::Time& time, const ros::Duration& period) override;

    void setDxLib(irsl_dynamixel::DynamixelShmPtr ptr) {
        dx_shm_ptr = ptr;
        this->setShmManager(ptr->shm_manager().get());
    }

public:
    irsl_dynamixel::DynamixelShmPtr dx_shm_ptr;
};

typedef std::shared_ptr<RobotDxHWShm> RobotDxHWShmPtr;

}  // namespace hardware_interface
