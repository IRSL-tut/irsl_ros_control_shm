#ifndef IRSL_print_urdf_H
#define IRSL_print_urdf_H

#include <string>
#include <iostream>

#include "joint_info.h"

void print_joint_urdf_header(std::ostream &os, const std::string &name);
void print_joint_urdf_footer(std::ostream &os);
void print_joint_urdf_joint(std::ostream &os, const std::string &jointName,
                            const std::string &interfaceType, const std::string &jointType,
                            float upper, float lower, float velocity, float effort);
void print_joint_urdf_joint(std::ostream &os, const joint_info &info);

#endif
