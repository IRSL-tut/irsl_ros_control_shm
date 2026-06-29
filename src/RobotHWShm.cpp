#include "RobotHWShm.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <stdexcept>
#include <unordered_map>

#include "irsl/shm_controller.h"

using namespace std;

namespace isc = irsl_shm_controller;
typedef std::vector<isc::irsl_float_type> floatvec;

namespace irsl_ros_control_shm {

namespace {

rclcpp::Logger logger()
{
    return rclcpp::get_logger("irsl_ros_control_shm");
}

uint64_t parseUnsigned(const std::unordered_map<std::string, std::string> &params,
                      const std::string &name,
                      uint64_t default_value)
{
    const auto it = params.find(name);
    if (it == params.end() || it->second.empty()) {
        return default_value;
    }

    return std::stoull(it->second);
}

bool isSupportedInterface(const std::string &interface_name)
{
    return interface_name == hardware_interface::HW_IF_POSITION ||
           interface_name == hardware_interface::HW_IF_VELOCITY ||
           interface_name == hardware_interface::HW_IF_EFFORT;
}

double *selectStateBuffer(const std::string &interface_name,
                          std::vector<double> &position,
                          std::vector<double> &velocity,
                          std::vector<double> &effort,
                          size_t index)
{
    if (interface_name == hardware_interface::HW_IF_POSITION) {
        return &position[index];
    }
    if (interface_name == hardware_interface::HW_IF_VELOCITY) {
        return &velocity[index];
    }
    if (interface_name == hardware_interface::HW_IF_EFFORT) {
        return &effort[index];
    }
    return nullptr;
}

}  // namespace

class RobotHWShm::Impl
{
public:
        Impl()
            : shm(nullptr),
                frame(0),
                hash(8888),
                shm_key(8888),
                has_position_command(false),
                has_velocity_command(false),
                has_effort_command(false)
    {
    }
    void initializeJoints(const std::vector<joint_info> &joints);
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);
public:
    std::unique_ptr<isc::ShmManager> owned_shm;
    isc::ShmManager *shm;

    std::vector<std::string> jointNames;
    std::vector<std::string> command_interfaces;
    std::vector<std::vector<std::string>> state_interfaces;
    std::vector<double> cur_pos;
    std::vector<double> cur_vel;
    std::vector<double> cur_eff;
    std::vector<double> com_pos;
    std::vector<double> com_vel;
    std::vector<double> com_eff;
    floatvec shm_cur_pos;
    floatvec shm_cur_vel;
    floatvec shm_cur_eff;
    floatvec shm_com_pos;
    floatvec shm_com_vel;
    floatvec shm_com_eff;

    uint64_t frame;
    uint64_t hash;
    uint32_t shm_key;
    bool has_position_command;
    bool has_velocity_command;
    bool has_effort_command;
};

RobotHWShm::RobotHWShm()
{
    impl = std::make_unique<Impl>();
}

RobotHWShm::~RobotHWShm() = default;

hardware_interface::CallbackReturn RobotHWShm::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    impl->hash = parseUnsigned(info_.hardware_parameters, "hash", 8888);
    impl->shm_key = static_cast<uint32_t>(parseUnsigned(info_.hardware_parameters, "shm_key", 8888));

    std::vector<joint_info> joints;
    joints.reserve(info_.joints.size());

    for (size_t index = 0; index < info_.joints.size(); ++index) {
        const auto &joint = info_.joints[index];
        joint_info joint_data;
        joint_data.name = joint.name;
        joint_data.index = static_cast<int>(index);
        joint_data.interfaceType = joint.command_interfaces.empty() ? hardware_interface::HW_IF_POSITION : joint.command_interfaces.front().name;
        joint_data.jointType = joint.type;
        joint_data.upper = 0.0;
        joint_data.lower = 0.0;
        joint_data.velocity = 0.0;
        joint_data.effort = 0.0;

        for (const auto &command_interface : joint.command_interfaces) {
            if (!isSupportedInterface(command_interface.name)) {
                RCLCPP_ERROR(logger(), "Unsupported command interface '%s' for joint '%s'", command_interface.name.c_str(), joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        for (const auto &state_interface : joint.state_interfaces) {
            if (!isSupportedInterface(state_interface.name)) {
                RCLCPP_ERROR(logger(), "Unsupported state interface '%s' for joint '%s'", state_interface.name.c_str(), joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        joints.push_back(joint_data);
    }

    if (initializeBackend() != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return initializeJoints(joints);
}

hardware_interface::CallbackReturn RobotHWShm::initializeJoints(const std::vector<joint_info> &joints)
{
    impl->initializeJoints(joints);

    auto settings = impl->owned_shm->settings();
    if (impl->has_position_command && settings.getOffsetPositionCommand() < 0) {
        RCLCPP_ERROR(logger(), "Position command interface requested but not available in shared memory");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (impl->has_velocity_command && settings.getOffsetVelocityCommand() < 0) {
        RCLCPP_ERROR(logger(), "Velocity command interface requested but not available in shared memory");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (impl->has_effort_command && settings.getOffsetTorqueCommand() < 0) {
        RCLCPP_ERROR(logger(), "Effort command interface requested but not available in shared memory");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

void RobotHWShm::Impl::initializeJoints(const std::vector<joint_info> &joints)
{
    size_t dof = joints.size();
    size_t maxIndex = 0;
    for(auto j = joints.begin(); j != joints.end(); j++) {
        if (static_cast<size_t>(j->index) >= maxIndex) {
            maxIndex = static_cast<size_t>(j->index);
        }
    }
    dof = std::max(dof, maxIndex + 1);
    cur_pos.resize(dof);
    cur_vel.resize(dof);
    cur_eff.resize(dof);
    com_pos.resize(dof);
    com_vel.resize(dof);
    com_eff.resize(dof);
    shm_cur_pos.resize(dof);
    shm_cur_vel.resize(dof);
    shm_cur_eff.resize(dof);
    shm_com_pos.resize(dof);
    shm_com_vel.resize(dof);
    shm_com_eff.resize(dof);
    jointNames.resize(dof);
    command_interfaces.resize(dof);
    state_interfaces.resize(dof);

    bool r0 = shm->readPositionCurrent(shm_cur_pos);
    if (r0) {
        for(size_t i = 0; i < dof; i++) {
            cur_pos[i] = shm_cur_pos[i];
            com_pos[i] = cur_pos[i];
        }
    }
    bool r1 = shm->readVelocityCurrent(shm_cur_vel);
    if (r1) {
        for(size_t i = 0; i < dof; i++) {
            cur_vel[i] = shm_cur_vel[i];
            com_vel[i] = cur_vel[i];
        }
    }
    bool r2 = shm->readTorqueCurrent(shm_cur_eff);
    if (r2) {
        for(size_t i = 0; i < dof; i++) {
            cur_eff[i] = shm_cur_eff[i];
            com_eff[i] = cur_eff[i];
        }
    }

    for (size_t i = 0; i < dof; ++i) {
        shm_com_pos[i] = com_pos[i];
        shm_com_vel[i] = com_vel[i];
        shm_com_eff[i] = com_eff[i];
    }

    for(auto j = joints.begin(); j != joints.end(); j++) {
        int idx = j->index;
        jointNames[idx] = j->name;
        command_interfaces[idx] = j->interfaceType;
        has_position_command = has_position_command || (j->interfaceType == hardware_interface::HW_IF_POSITION);
        has_velocity_command = has_velocity_command || (j->interfaceType == hardware_interface::HW_IF_VELOCITY);
        has_effort_command = has_effort_command || (j->interfaceType == hardware_interface::HW_IF_EFFORT);
        state_interfaces[idx] = {
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT,
        };
    }
}

void RobotHWShm::setShmManager(isc::ShmManager *ptr)
{
    impl->shm = ptr;
}

hardware_interface::CallbackReturn RobotHWShm::initializeBackend()
{
    isc::ShmSettings settings;
    settings.hash = impl->hash;
    settings.shm_key = impl->shm_key;

    impl->owned_shm = std::make_unique<isc::ShmManager>(settings);
    const bool opened = impl->owned_shm->openSharedMemory(false);
    if (!opened || !impl->owned_shm->isOpen()) {
        RCLCPP_ERROR(logger(), "Failed to open shared memory (hash=%llu, shm_key=%u)",
                     static_cast<unsigned long long>(impl->hash), impl->shm_key);
        impl->owned_shm.reset();
        impl->shm = nullptr;
        return hardware_interface::CallbackReturn::ERROR;
    }

    setShmManager(impl->owned_shm.get());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHWShm::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t index = 0; index < info_.joints.size(); ++index) {
        for (const auto &state_interface : info_.joints[index].state_interfaces) {
            double *buffer = selectStateBuffer(state_interface.name, impl->cur_pos, impl->cur_vel, impl->cur_eff, index);
            if (buffer != nullptr) {
                interfaces.emplace_back(info_.joints[index].name, state_interface.name, buffer);
            }
        }
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotHWShm::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t index = 0; index < info_.joints.size(); ++index) {
        for (const auto &command_interface : info_.joints[index].command_interfaces) {
            double *buffer = selectStateBuffer(command_interface.name, impl->com_pos, impl->com_vel, impl->com_eff, index);
            if (buffer != nullptr) {
                interfaces.emplace_back(info_.joints[index].name, command_interface.name, buffer);
            }
        }
    }
    return interfaces;
}

hardware_interface::CallbackReturn RobotHWShm::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    if (impl->shm == nullptr) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHWShm::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotHWShm::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    return impl->read(time, period);
}

hardware_interface::return_type RobotHWShm::Impl::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    (void)time;
    (void)period;
    if (shm == nullptr) {
        return hardware_interface::return_type::ERROR;
    }

    const bool position_ok = shm->readPositionCurrent(shm_cur_pos);
    const bool velocity_ok = shm->readVelocityCurrent(shm_cur_vel);
    const bool effort_ok = shm->readTorqueCurrent(shm_cur_eff);

    if (!(position_ok && velocity_ok && effort_ok)) {
        return hardware_interface::return_type::ERROR;
    }

    for (size_t index = 0; index < cur_pos.size(); ++index) {
        cur_pos[index] = shm_cur_pos[index];
        cur_vel[index] = shm_cur_vel[index];
        cur_eff[index] = shm_cur_eff[index];
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHWShm::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    return impl->write(time, period);
}

hardware_interface::return_type RobotHWShm::Impl::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    (void)time;
    (void)period;
    if (shm == nullptr) {
        return hardware_interface::return_type::ERROR;
    }

    for (size_t index = 0; index < com_pos.size(); ++index) {
        shm_com_pos[index] = com_pos[index];
        shm_com_vel[index] = com_vel[index];
        shm_com_eff[index] = com_eff[index];
    }

    const bool position_ok = !has_position_command || shm->writePositionCommand(shm_com_pos);
    const bool velocity_ok = !has_velocity_command || shm->writeVelocityCommand(shm_com_vel);
    const bool effort_ok = !has_effort_command || shm->writeTorqueCommand(shm_com_eff);

    frame++;
    return (position_ok && velocity_ok && effort_ok) ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

}  // namespace irsl_ros_control_shm

PLUGINLIB_EXPORT_CLASS(irsl_ros_control_shm::RobotHWShm, hardware_interface::SystemInterface)
