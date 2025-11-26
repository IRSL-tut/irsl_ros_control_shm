#include "RobotDxHWShm.h"

using namespace std;

namespace hardware_interface {

void RobotDxHWShm::read(const ros::Time& time, const ros::Duration& period)
{
    dx_shm_ptr->writeDx();
    RobotHWShm::read(time, period);
#if 0
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
#endif
}

void RobotDxHWShm::write(const ros::Time& time, const ros::Duration& period)
{
    dx_shm_ptr->readDx();
    RobotHWShm::write(time, period);
#if 0
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
#endif
}

}
