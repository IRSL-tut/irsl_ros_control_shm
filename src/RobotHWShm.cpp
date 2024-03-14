#include "RobotHWShm.h"

//#include <angles/angles.h>
//#include <pluginlib/class_list_macros.h>
//#include <transmission_interface/transmission_parser.h>
//#include <joint_limits_interface/joint_limits_rosparam.h>
//#include <joint_limits_interface/joint_limits_urdf.h>
//#include <unordered_map>
//#include <limits>

using namespace std;

namespace hardware_interface {

class RobotHWShm::Impl
{
public:
    Impl() : shm(nullptr), frame(0)
    {
    }

public:
    ShmManager *shm;

    std::vector<std::string> jointNames;
    std::vector<irsl_float_type> cur_pos;
    std::vector<irsl_float_type> cur_vel;
    std::vector<irsl_float_type> cur_eff;
    std::vector<irsl_float_type> com_pos;
    std::vector<irsl_float_type> com_vel;
    std::vector<irsl_float_type> com_eff;

    // Interface //
    hardware_interface::JointStateInterface    jointStateInterface;
    hardware_interface::PositionJointInterface positionJointInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;
    hardware_interface::EffortJointInterface   effortJointInterface;

    uint64_t frame;
};

RobotHWShm::RobotHWShm()
{
    impl = new Impl();
}


void RobotHWShm::initializeJoints(std::vector<joint_info> &joints)
{
    int dof = joints.size();
    int maxIndex = 0;
    for(auto j = joints.begin(); j != joints.end(); j++) {
        if (j->index >= maxIndex) {
            maxIndex = j->index;
        }
    }
    dof = maxIndex > dof ? maxIndex : dof;
    std::cerr << "dof : " << dof << std::endl;
    cur_pos.resize(dof);
    cur_vel.resize(dof);
    cur_eff.resize(dof);

    com_pos.resize(dof);
    com_vel.resize(dof);
    com_eff.resize(dof);

    for(auto j = joints.begin(); j != joints.end(); j++) {
        int idx = j->index;
        jointStateInterface.registerHandle(
            hardware_interface::JointStateHandle(j->name, &(cur_pos[idx]), &(cur_vel[idx]), &(cur_eff[idx])) );
        if (j->interfaceType == "Position") {
            // Joint Handle //
            hardware_interface::JointHandle jointHandle;
            jointHandle = hardware_interface::JointHandle(jointStateInterface.getHandle(j->name), &(com_pos[idx]));
            positionJointInterface.registerHandle(jointHandle);
        }
        if (j->interfaceType == "Velocity") {
            // Joint Handle //
            hardware_interface::JointHandle jointHandle;
            jointHandle = hardware_interface::JointHandle(jointStateInterface.getHandle(j->name), &(com_vel[idx]));
            velocityJointInterface.registerHandle(jointHandle);
        }
        if (j->interfaceType == "Effort") {
            // Joint Handle //
            hardware_interface::JointHandle jointHandle;
            jointHandle = hardware_interface::JointHandle(jointStateInterface.getHandle(j->name), &(com_eff[idx]));
            effortJointInterface.registerHandle(jointHandle);
        }
    }
    registerInterface(&jointStateInterface);
    registerInterface(&positionJointInterface);
    registerInterface(&velocityJointInterface);
    registerInterface(&effortJointInterface);
}

void RobotHWShm::setShmManager(ShmManager *ptr)
{
    impl->shm = ptr;
}

void RobotHWShm::read(const ros::Time& time, const ros::Duration& period)
{
    if (!!(impl->shm)) {
        impl->shm->readPositionCurrent(cur_pos);
        impl->shm->readVelocityCurrent(cur_vel);
        impl->shm->readTorqueCurrent(cur_eff);
    }
}

void RobotHWShm::write(const ros::Time& time, const ros::Duration& period)
{
    bool res;
    if (!!(impl->shm)) {
#if 0
        std::cerr << "w(" << impl->frame << ") " << com_pos[0] << std::endl;
        res = impl->shm->writePositionCommand(com_pos);
        if(!res) {
            std::cerr << "fail" << std::endl;
        }
#endif
        impl->shm->writePositionCommand(com_pos);
        impl->shm->writeVelocityCommand(com_vel);
        impl->shm->writeTorqueCommand(com_eff);
        //
        res = impl->shm->setFrame(impl->frame);
        res = impl->shm->setTime(time.sec, time.nsec);
#if 0
        std::vector<double> vec(6);
        res = impl->shm->readPositionCommand(vec);
        if (res) {
            std::cerr << "v : " << vec[0] << std::endl;
        }
#endif
    }
    impl->frame++;
}

}  // namespace hardware_interface

//PLUGINLIB_EXPORT_CLASS(hardware_interface::RobotHWShm, hardware_interface::RobotHWShm)
