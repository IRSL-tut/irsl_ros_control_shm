#ifndef IRSL_joint_info_H
#define IRSL_joint_info_H

#include <string>

struct joint_info
{
    std::string name;
    std::string interfaceType; // Position, Velocity, Effort
    std::string jointType; // revolute, continuous, prismatic, fixed, free
    // limits
    float upper;
    float lower;
    float velocity;
    float effort;

    int index;
};

#endif
