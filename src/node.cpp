#include <ros/ros.h>
#include <ros/spinner.h>
////
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>

#include "RobotHWShm.h"
#include "print_urdf.h"

#include "irsl/shm_controller.h"
#include "irsl/realtime_task.h"
#include "irsl/thirdparty/CLI11.hpp"

namespace isc = irsl_shm_controller;
namespace irt = irsl_realtime_task;

class OptParse : public CLI::App
{
private:
    uint64_t _hash;
    uint32_t _key;
public:
    OptParse(const std::string name) : CLI::App(name)
    {
        add_option("--hash", _hash, "")->default_val("8888");
        add_option("--shm_key", _key, "")->default_val("8888");
        allow_extras(true);
    }
    uint64_t getHash() { return _hash; }
    uint32_t getShmKey() { return _key; }
};

int main(int argc, char **argv)
{
    boost::shared_ptr<hardware_interface::RobotHWShm> robotHWShm;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;

    //// ROS initialization
    ros::init(argc, argv, "node_shm");
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");

    OptParse op("test");
    op.parse(argc, argv);

    std::vector<std::string> jointnames;
    if (nodeHandle.getParam("jointnames", jointnames)) {
        ROS_INFO("Successfully got jointnames");
    } else {
        ROS_ERROR("Failed to get param 'jointnames'");
        return 0;
    }

    std::vector<joint_info> joint_settings;
    {   // joint_settings may be generated from .body or .urdf, etc.
        // TODO:
        joint_settings.resize(jointnames.size());
        int cntr = 0;
        for(auto s = joint_settings.begin(); s != joint_settings.end(); s++) {
            s->index = cntr++;
            s->upper =  6;
            s->lower = -6;
            s->velocity = 10;
            s->effort   = 100;
            s->interfaceType = "Position";
            s->jointType = "revolute";
        }

        for (size_t i=0; i<jointnames.size();i++)
        {
            joint_settings[i].name = jointnames[i];
            std::cout << i << " : "  << joint_settings[i].name << std::endl;
        }
    }

    {   // write dummy urdf for trajectory_confroller
        std::ostringstream oss;
        print_joint_urdf_header(oss, "DummyRobot"); // TODO: name of robot
        for(auto s = joint_settings.begin(); s != joint_settings.end(); s++) {
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
            {   //// shared memory settings
                isc::ShmSettings ss;
                ss.hash    = op.getHash();
                ss.shm_key = op.getShmKey();
                isc::ShmManager *sm = new isc::ShmManager(ss);
                bool res;
                res = sm->openSharedMemory(false); // open as client
                std::cerr << "shm open: " << res << std::endl;
                std::cout << "shm checkHeader() : " << sm->checkHeader() << std::endl;
                // TODO : check compatibility joint_settings and shm-settings
                res = sm->isOpen();
                std::cout << "shm isOpen: " << res << std::endl;
                if (!res) {
                    // exit
                    return 0;
                }
                robotHWShm->setShmManager(sm);
            }
            robotHWShm->initializeJoints(joint_settings);

            if(!controllerManager){
                controllerManager = std::make_shared<controller_manager::ControllerManager>(robotHWShm.get(), nodeHandle);
            }

            //irt::RealtimeContext rt(5000000); // 5ms TODO
            irt::RealtimeContext rt(1000000000); // 1000ms TODO
            rt.start();

            int cntr = 0;
            ros::Time previous = ros::Time::now();
            rt.waitNextFrame();
            while(ros::ok()) {  //// realtime-loop
                ros::Time now        = ros::Time::now();
                ros::Duration period = now - previous;
                previous = now;
                ////
                robotHWShm->read(now, period);
                robotHWShm->write(now, period);
                controllerManager->update(now, period, false); // TODO: false
                ////
                cntr++;
                if (cntr > 100) {
                    std::cerr << "max: " << rt.getMaxInterval() << std::endl;
                    rt.reset();
                    cntr = 0;
                }
                //// wait
                rt.waitNextFrame();
            }
        } else {
            std::cout << "HW is not initialided" << std::endl;
        }
    } else {
        std::cout << "ROS is not initialided" << std::endl;
    }
    // TODO: finalize
    return 0;
}
