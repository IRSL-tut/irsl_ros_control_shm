#include <ros/ros.h>
#include <ros/spinner.h>
////
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>

#include "irsl/realtime_task.h"

#include "RobotHWShm.h"
#include "print_urdf.h"

int main(int argc, char **argv)
{
    ////
    boost::shared_ptr<hardware_interface::RobotHWShm> robotHWShm;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;

    //// ROS initialization
    ros::init(argc, argv, "node_shm");
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");
    //ros::NodeHandle nodeHandle = ros::NodeHandle("irsl_test");

    std::vector<joint_info> settings;
    {   // settings may be generated from .body or .urdf, etc.
        // TODO:
        settings.resize(3);
        int cntr = 0;
        for(auto s = settings.begin(); s != settings.end(); s++) {
            s->index = cntr++;
            s->upper =  6;
            s->lower = -6;
            s->velocity = 10;
            s->effort   = 100;
            s->interfaceType = "Position";
            s->jointType = "revolute";
        }
        settings[0].name = "joint0";
        settings[1].name = "joint1";
        settings[2].name = "joint2";
    }

    {   // write dummy urdf for trajectory_confroller
        std::ostringstream oss;
        print_joint_urdf_header(oss, "dummyRobot");
        for(auto s = settings.begin(); s != settings.end(); s++) {
            print_joint_urdf_joint(oss, *s);
        }
        print_joint_urdf_footer(oss);
        // ros-param
        ros::NodeHandle robo_desc;
        robo_desc.setParam("robot_description", oss.str());
    }

    std::unique_ptr<ros::AsyncSpinner> spinner;

    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();

    if(ros::isInitialized()) {
        robotHWShm = boost::make_shared<hardware_interface::RobotHWShm>();

        if(!!robotHWShm) {
            {
                ShmSettings ss;
                ss.numJoints = 3;
                ss.numForceSensors = 1;
                ss.numImuSensors   = 1;
                ss.hash    = 8888;
                ss.shm_key = 8888;
                //ss.extraDataSize = 0;
                //ss.extraDataSize = 96;
                ss.jointType = ShmSettings::PositionCommand | ShmSettings::PositionGains;

                ShmManager *sm = new ShmManager(ss);

                bool res;
                res = sm->openSharedMemory(false);
                std::cerr << "shm open: " << res << std::endl;
                std::cout << "shm checkHeader() : " << sm->checkHeader() << std::endl;

                res = sm->isOpen();
                std::cout << "shm isOpen: " << res << std::endl;
                if (!res) {
                    // exit
                    return 0;
                }
                robotHWShm->setShmManager(sm);
            }
            robotHWShm->initializeJoints(settings);

            if(!controllerManager){
                controllerManager = std::make_shared<controller_manager::ControllerManager>(robotHWShm.get(), nodeHandle);
            }

            //// For realtime
            irsl_realtime_task::IntervalStatistics tm(10000);
            tm.start();

            int cntr = 0;
            ros::Time previous = ros::Time::now();
            while(true) {
                //// loop
                ros::Time now        = ros::Time::now();
                ros::Duration period = now - previous;
                previous = now;

                ////
                robotHWShm->read(now, period);
                robotHWShm->write(now, period);
                controllerManager->update(now, period, false); //// TODO: false

                //// wait
                tm.sleepUntil(10000000);
                tm.sync();
                cntr++;
                if (cntr > 100) {
                    std::cerr << "max: " << tm.getMaxInterval() << std::endl;
                    tm.reset();
                    cntr = 0;
                }
            }
        } else {
            std::cout << "HW is not initialided" << std::endl;
        }
    } else {
        std::cout << "ROS is not initialided" << std::endl;
    }
    return 0;
}
