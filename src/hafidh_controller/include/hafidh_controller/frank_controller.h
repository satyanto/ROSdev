/* Hafidh Satyanto */

// Here we'll declare the actual controller class in our namespace scope.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

// We're going to use the default compliance parameters.
#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace hafidh_controller {
    class FrankController : public controller_interface::MultiInterfaceController<
                                franka_hw::FrankaModelInterface,
                                hardware_interface::EffortJointInterface,
                                franka_hw::FrankaStateInterface> {
        public:
            bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
            void starting(const ros::Time&) override;
            void update(const ros::Time&, const ros::Duration& period) override;

        private:
            Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                const Eigen::Matrix<double, 7, 1>& tau_J_d);
            
            std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
            std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
            std::vector<hardware_interface::JointHandle> joint_handles_;

            double filter_params_{0.005};
            const double delta_tau_max_{1.0};

            double nullspace_stiffness_{20.0};
            double nullspace_stiffness_target_{20.0};

            Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
            Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;

            Eigen::Matrix<double, 6, 6> cartesian_damping_;
            Eigen::Matrix<double, 6, 6> cartesian_damping_target_;

            Eigen::Matrix<double, 7, 1> q_d_nullspace_;

            Eigen::Vector3d position_d_;
            Eigen::Quaterniond orientation_d_;
            Eigen::Vector3d position_d_target_;
            Eigen::Quaterniond orientation_d_target_;

            // Dynamic reconfigure for compliance parameters... I'm going to leave it as-is.
            std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>> dynamic_server_compliance_param_;
            ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
            void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config, uint32_t level);

            // Our target pose subscriber
            ros::Subscriber sub_target_pose_;
            void targetPoseHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    };
}
