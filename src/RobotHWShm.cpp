#include "RobotHWShm.h"
#include "irsl/shm_controller.h"

//#include <angles/angles.h>
//#include <pluginlib/class_list_macros.h>
//#include <transmission_interface/transmission_parser.h>
//#include <joint_limits_interface/joint_limits_rosparam.h>
//#include <joint_limits_interface/joint_limits_urdf.h>
//#include <unordered_map>
//#include <limits>

using namespace std;

namespace isc = irsl_shm_controller;
typedef std::vector<isc::irsl_float_type> floatvec;

namespace hardware_interface {

class RobotHWShm::Impl
{
public:
    Impl() : shm(nullptr), frame(0)
    {
    }

public:
    isc::ShmManager *shm;

    std::vector<std::string> jointNames;
    floatvec cur_pos;
    floatvec cur_vel;
    floatvec cur_eff;
    floatvec com_pos;
    floatvec com_vel;
    floatvec com_eff;

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
    impl->cur_pos.resize(dof);
    impl->cur_vel.resize(dof);
    impl->cur_eff.resize(dof);

    impl->com_pos.resize(dof);
    impl->com_vel.resize(dof);
    impl->com_eff.resize(dof);

    for(auto j = joints.begin(); j != joints.end(); j++) {
        int idx = j->index;
        jointStateInterface.registerHandle(
            hardware_interface::JointStateHandle(j->name, &(impl->cur_pos[idx]), &(impl->cur_vel[idx]), &(impl->cur_eff[idx])) );
        if (j->interfaceType == "Position") {
            // Joint Handle //
            hardware_interface::JointHandle jointHandle;
            jointHandle = hardware_interface::JointHandle(jointStateInterface.getHandle(j->name), &(impl->com_pos[idx]));
            positionJointInterface.registerHandle(jointHandle);
        }
        if (j->interfaceType == "Velocity") {
            // Joint Handle //
            hardware_interface::JointHandle jointHandle;
            jointHandle = hardware_interface::JointHandle(jointStateInterface.getHandle(j->name), &(impl->com_vel[idx]));
            velocityJointInterface.registerHandle(jointHandle);
        }
        if (j->interfaceType == "Effort") {
            // Joint Handle //
            hardware_interface::JointHandle jointHandle;
            jointHandle = hardware_interface::JointHandle(jointStateInterface.getHandle(j->name), &(impl->com_eff[idx]));
            effortJointInterface.registerHandle(jointHandle);
        }
    }
    registerInterface(&jointStateInterface);
    registerInterface(&positionJointInterface);
    registerInterface(&velocityJointInterface);
    registerInterface(&effortJointInterface);
}

void RobotHWShm::setShmManager(isc::ShmManager *ptr)
{
    impl->shm = ptr;
    //// Wait frame
    //// set CurPos -> ComPos
}

void RobotHWShm::read(const ros::Time& time, const ros::Duration& period)
{
    if (!!(impl->shm)) {
        bool r0 = impl->shm->readPositionCurrent(impl->cur_pos);
        bool r1 = impl->shm->readVelocityCurrent(impl->cur_vel);
        bool r2 = impl->shm->readTorqueCurrent(impl->cur_eff);
#if 0
        std::cerr << "rd " << r0 << ", " << r1 << ", " << r1 << std::endl;
        std::cerr << impl->cur_pos[0] << std::endl;
        std::cerr << impl->cur_vel[0] << std::endl;
        std::cerr << impl->cur_eff[0] << std::endl;
#endif
    }
}

void RobotHWShm::write(const ros::Time& time, const ros::Duration& period)
{
    bool res;
    if (!!(impl->shm)) {
#if 0
        std::cerr << "w(" << impl->frame << ") " << impl->com_pos[0] << std::endl;
        res = impl->shm->writePositionCommand(impl->com_pos);
        if(!res) {
            std::cerr << "fail" << std::endl;
        }
#endif
        impl->shm->writePositionCommand(impl->com_pos);
        impl->shm->writeVelocityCommand(impl->com_vel);
        impl->shm->writeTorqueCommand(impl->com_eff);
        //
        //res = impl->shm->setFrame(impl->frame);
        //res = impl->shm->setTime(time.sec, time.nsec);
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
