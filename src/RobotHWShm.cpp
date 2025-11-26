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
    void initializeJoints(const std::vector<joint_info> &joints);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);
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

void RobotHWShm::initializeJoints(const std::vector<joint_info> &joints)
{
    impl->initializeJoints(joints);

    registerInterface(&(impl->jointStateInterface));
    registerInterface(&(impl->positionJointInterface));
    registerInterface(&(impl->velocityJointInterface));
    registerInterface(&(impl->effortJointInterface));
}

void RobotHWShm::Impl::initializeJoints(const std::vector<joint_info> &joints)
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
    //
    bool r0 = shm->readPositionCurrent(cur_pos);
    if (r0) {
        for(int i = 0; i < dof; i++) {
            com_pos[i] = cur_pos[i];
        }
    }
    bool r1 = shm->readVelocityCurrent(cur_vel);
    if (r1) {
        for(int i = 0; i < dof; i++) {
            com_vel[i] = cur_vel[i];
        }
    }
    bool r2 = shm->readTorqueCurrent(cur_eff);
    if (r2) {
        for(int i = 0; i < dof; i++) {
            com_eff[i] = cur_eff[i];
        }
    }
    //
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
}

void RobotHWShm::setShmManager(isc::ShmManager *ptr)
{
    impl->shm = ptr;
    //// Wait frame
    //// set CurPos -> ComPos
}

void RobotHWShm::read(const ros::Time& time, const ros::Duration& period)
{
    impl->read(time, period);
}
void RobotHWShm::Impl::read(const ros::Time& time, const ros::Duration& period)
{
    if (!!(shm)) {
        bool r0 = shm->readPositionCurrent(cur_pos);
        bool r1 = shm->readVelocityCurrent(cur_vel);
        bool r2 = shm->readTorqueCurrent(cur_eff);
#if 0
        std::cerr << "rd " << r0 << ", " << r1 << ", " << r1 << std::endl;
        std::cerr << cur_pos[0] << std::endl;
        std::cerr << cur_vel[0] << std::endl;
        std::cerr << cur_eff[0] << std::endl;
#endif
    }
}
void RobotHWShm::write(const ros::Time& time, const ros::Duration& period)
{
    impl->write(time, period);
}
void RobotHWShm::Impl::write(const ros::Time& time, const ros::Duration& period)
{
    bool res;
    if (!!(shm)) {
#if 0
        std::cerr << "w(" << frame << ") " << com_pos[0] << std::endl;
        res = shm->writePositionCommand(com_pos);
        if(!res) {
            std::cerr << "fail" << std::endl;
        }
#endif
        shm->writePositionCommand(com_pos);
        shm->writeVelocityCommand(com_vel);
        shm->writeTorqueCommand(com_eff);
        //
        //res = shm->setFrame(frame);
        //res = shm->setTime(time.sec, time.nsec);
#if 0
        std::vector<double> vec(6);
        res = shm->readPositionCommand(vec);
        if (res) {
            std::cerr << "v : " << vec[0] << std::endl;
        }
#endif
    }
    frame++;
}

}  // namespace hardware_interface

//PLUGINLIB_EXPORT_CLASS(hardware_interface::RobotHWShm, hardware_interface::RobotHWShm)
