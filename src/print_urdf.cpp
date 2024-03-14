#include "print_urdf.h"
#include <limits>

void print_joint_urdf_header(std::ostream &os, const std::string &name)
{
    const std::string rootName("root");

    os << "<?xml version=\"1.0\" ?>" << std::endl;
    os << "<robot name=\"" << name << "\">" << std::endl;
    os << "<link name=\"" << rootName << "\"/>" << std::endl;
}
void print_joint_urdf_footer(std::ostream &os)
{
    os << "</robot>" << std::endl;
}
void print_joint_limit(std::ostream &os, const std::string _offset, float upper, float lower, float velocity, float effort)
{
    os << _offset << "<limit lower=\"" << lower << "\" upper=\"" << upper << "\" velocity=\"" << velocity << "\" effort=\"" << effort << "\" />" << std::endl;
}
void print_joint_urdf_joint(std::ostream &os, const std::string &jointName, const std::string &interfaceType, const std::string &jointType,
                            float upper, float lower, float velocity, float effort)
// interfaceType: 'Position', 'Effort', 'Velocity'
// jointType: revolute, continuous, prismatic, fixed, floating, planar
{
    const std::string rootName("root");

    // local_link
    os << "<link name=\"link_" << jointName << "\" />" << std::endl; // jname
    // joint(root->local_link)
    os << "<joint name=\"" << jointName << "\" type=\"" << jointType << "\">" << std::endl; // jname
    os << "  <parent link=\"root\"/>" << std::endl;
    os << "  <child  link=\"link_" << jointName << "\"/>" << std::endl;
    //
    print_joint_limit(os, "  ", upper, lower, velocity, effort);
    os << "</joint>" << std::endl;
    //// for ros_control
    os << "<transmission name=\"" << jointName << "_trans\">" << std::endl;
    os << "  <type>transmission_interface/SimpleTransmission</type>" << std::endl;
    os << "  <joint name=\"" << jointName << "\">" << std::endl;
    os << "    <hardwareInterface>hardware_interface/" << interfaceType << "JointInterface</hardwareInterface>" << std::endl;
    os << "  </joint>" << std::endl;
    os << "  <actuator name=\"" << jointName << "_motor\">" << std::endl;
    os << "    <mechanicalReduction>1</mechanicalReduction>" << std::endl;
    os << "  </actuator>" << std::endl;
    os << "</transmission>" << std::endl;
}

void print_joint_urdf_joint(std::ostream &os, const joint_info &info)
{
    print_joint_urdf_joint(os, info.name, info.interfaceType, info.jointType,
                           info.upper, info.lower, info.velocity, info.effort);
}

#if 0
#include <vector>
#include <sstream>

int main(int argc, char **argv)
{
    std::vector<joint_info> jlist;
    {
        joint_info j;
        j.name = "joint0";
        j.interfaceType = "Position";
        j.jointType = "revolute";
        j.upper =  6;
        j.lower = -6;
        j.velocity = 10;
        j.effort   = 100;
        jlist.push_back(j);
    }
    {
        joint_info j;
        j.name = "joint1";
        j.interfaceType = "Position";
        j.jointType = "revolute";
        j.upper =  6;
        j.lower = -6;
        j.velocity = 10;
        j.effort   = 100;
        jlist.push_back(j);
    }
    {
        joint_info j;
        j.name = "joint2";
        j.interfaceType = "Position";
        j.jointType = "revolute";
        j.upper =  6;
        j.lower = -6;
        j.velocity = 10;
        j.effort   = 100;
        jlist.push_back(j);
    }
    std::ostringstream oss;
    print_joint_urdf_header(oss, "dummyRobot");
    for(auto j = jlist.begin(); j != jlist.end(); j++) {
        print_joint_urdf_joint(oss, j->name, j->interfaceType, j->jointType, j->upper, j->lower, j->velocity, j->effort);
    }
    print_joint_urdf_footer(oss);

    std::cout << oss.str();
}
#endif
