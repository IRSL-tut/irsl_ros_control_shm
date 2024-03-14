#ifndef ROBOT_HW_SHM_H
#define ROBOT_HW_SHM_H

#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

//#include <transmission_interface/transmission_info.h>
//#include <urdf/model.h>
#include <vector>
#include <iostream>

#include "joint_info.h"
#include "irsl/shm_controller.h"

using namespace irsl_shm_controller;

namespace hardware_interface {

class RobotHWShm : public RobotHW
{
public:
    RobotHWShm(); //// shm settings

    //virtual bool initSim(const ros::NodeHandle& nh, cnoid::ControllerIO* args) final;
    virtual bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &/*robot_hw_nh*/) {
        std::cerr << "init:" << std::endl;
        return true;
    }
    virtual void read(const ros::Time& time, const ros::Duration& period) override;
    virtual void write(const ros::Time& time, const ros::Duration& period) override;

    void setShmManager(ShmManager *ptr);
    void initializeJoints(std::vector<joint_info> &joints);

    //// may move to impl
    std::vector<std::string> jointNames;
    std::vector<double> cur_pos;
    std::vector<double> cur_vel;
    std::vector<double> cur_eff;
    std::vector<double> com_pos;
    std::vector<double> com_vel;
    std::vector<double> com_eff;

    // Interface //
    hardware_interface::JointStateInterface    jointStateInterface;
    hardware_interface::PositionJointInterface positionJointInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;
    hardware_interface::EffortJointInterface   effortJointInterface;

private:
    class Impl;
    Impl *impl;
};

typedef std::shared_ptr<RobotHWShm> RobotHWShmPtr;

}  // namespace hardware_interface

#endif  // ROBOT_HW_SHM_H
