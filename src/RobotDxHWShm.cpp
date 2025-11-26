#include "RobotDxHWShm.h"

using namespace std;

namespace hardware_interface {

void RobotDxHWShm::read(const ros::Time& time, const ros::Duration& period)
{
    dx_shm_ptr->writeDx();
    RobotHWShm::read(time, period);
}

void RobotDxHWShm::write(const ros::Time& time, const ros::Duration& period)
{
    dx_shm_ptr->readDx();
    RobotHWShm::write(time, period);
}

}
