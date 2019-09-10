// Hafidh's 2nd attempt sad!

#include <hafidh_controller/hafidh_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>    // what is this for??
#include <ros/ros.h>

namespace hafidh_controller {
    bool HafidhController::init(hardware_interface::RobotHW* robot_hardware,
                                ros::NodeHandle& node_handle) {
        cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
        if (cartesian_pose_interface_ == nullptr) {
            ROS_ERROR(
                "HafidhController: Could not get Cartesian Pose "
                "interface from hardware!");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("HafidhController: Could not get parameter arm_id");
            return false;
        }

        try {
            cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
                cartesian_pose_interface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "HafidhController: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("HafidhController: Could not get state interface from hardware");
            return false;
        }

        try {
            auto state_handle = state_interface->getHandle(arm_id + "_robot");

            std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            for (size_t i = 0; i < q_start.size(); i++) {
                if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
                    ROS_ERROR_STREAM(
                        "HafidhController: Robot is not in the expected starting position for "
                        "running this example.");
                    return false;
                }
            }
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "HafidhController: Exception getting state handle: " << e.what());
            return false;
        }

        return true;
    }

    void HafidhController::starting(const ros::Time&) {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        elapsed_time_ = ros::Duration(0.0);
    }

    void HafidhController::update(const ros::Time&,
                                                const ros::Duration& period) {
        elapsed_time_ += period;

        double radius = 0.3;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
        double delta_x = radius * std::sin(angle);
        double delta_z = radius * (std::cos(angle) - 1);
        std::array<double, 16> new_pose = initial_pose_;
        new_pose[12] -= delta_x;
        new_pose[14] -= delta_z;
        cartesian_pose_handle_->setCommand(new_pose);
    }

}   // namespace hafidh_controller

PLUGINLIB_EXPORT_CLASS(hafidh_controller::HafidhController,
                        controller_interface::ControllerBase)