#include <ros/ros.h>
#include <ros/spinner.h>
////
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>

#include "RobotDxHWShm.h"
#include "print_urdf.h"

#include "irsl/shm_controller.h"
#include "irsl/realtime_task.h"
#include "irsl/opt_parse.hpp"

namespace icu = irsl_common_utils;
namespace isc = irsl_shm_controller;
namespace irt = irsl_realtime_task;

int main(int argc, char **argv)
{
    boost::shared_ptr<hardware_interface::RobotDxHWShm> robotHWShm;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;

    //// ROS initialization
    ros::init(argc, argv, "node_dx_shm");
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");

    std::string fname;
    bool verbose = false;
    icu::OptParse op("node_dx_shm");
    op.add_option("--config", fname, "name of input file(.yaml)")->default_val("config.yaml");
    op.add_flag("-v,--verbose", verbose, "verbose message");
    op.opt_parse(argc, argv);

    std::vector<std::string> jointnames;
    // TODO: jointnames reading from config
    if (nodeHandle.getParam("jointnames", jointnames)) {
        ROS_INFO("Successfully got jointnames");
    } else {
        ROS_ERROR("Failed to get param 'jointnames'");
        return 0;
    }
    double period = 0.01;
    nodeHandle.getParam("period", period);
    ROS_INFO("period: %f", period);

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
        if (verbose) {
            std::cerr << "init: robotHWShm" << std::endl;
        }
        robotHWShm = boost::make_shared<hardware_interface::RobotDxHWShm>();

        if(!!robotHWShm) {
            {
                irsl_dynamixel::DynamixelShmPtr ds(new irsl_dynamixel::DynamixelShm());
                ROS_INFO_STREAM( "ds->initialize : " << fname  );
                ds->initialize(fname, op.getHash(), op.getShmKey());
                //
                ds->readDx();
                ds->initializeCommand();
                //
                robotHWShm->setDxLib(ds);
            }
            if (verbose) {
                std::cerr << "initialieJoints" << std::endl;
            }
            robotHWShm->initializeJoints(joint_settings);

            if(!controllerManager){
                controllerManager = std::make_shared<controller_manager::ControllerManager>(robotHWShm.get(), nodeHandle);
            }

            unsigned long period_ns = 1000000000 * period;
            irt::RealtimeContext rt(period_ns);
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
                    if (verbose) {
                        std::cerr << "max: " << rt.getMaxInterval() << std::endl;
                    }
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
    ros::shutdown();
    // TODO: finalize
    return 0;
}
