/* Hafidh Satyanto */
// Taken from Franka Emika's Cartesian Impedance Example Controller, I rewrote line by line to get an understanding
// Here's to those who name stuff. Yeet! Frank! Kip!

// Our controller library include
#include <hafidh_controller/frank_controller.h>

// C++ includes (?)
#include <cmath>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

// Utility code includes
#include "pseudo_inversion.h"

namespace hafidh_controller {
    bool FrankController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        sub_target_pose_ = node_handle.subscribe("/target_pose", 20, &FrankController::targetPoseHandler, this, ros::TransportHints().reliable().tcpNoDelay());
    
        std::string arm_id = "panda";
        std::vector<std::string> joint_names = {
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7"
        };

        // We are trying to get the robot model interface -- set as a pointer? (auto?)
        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("FrankController: Error getting model interface from robot_hw");
            return false;
        }

        // We are trying to get the robot model handle (handler?)
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("FrankController: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        // We are trying to get the robot state interface
        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM("FrankController: Error getting state interface from robot_hw");
            return false;
        }

        // We are trying to get the robot state handle (handler?) (need to look into)
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("FrankController: Exception getting state handle from interface: " << ex.what());
            return false;
        }

        // Look into effort joint interface... (torque sensor at joints is effort?)
        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("FrankController: Error getting effort joint interface from robot_hw");
            return false;
        }

        // Get the joint handles... (but what is pushback?)
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM("FrankController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        // Get our robot parameters (in this case just using the default Franka ones)
        dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");
        dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<hafidh_controller::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
        dynamic_server_compliance_param_->setCallback(boost::bind(&FrankController::complianceParamCallback, this, _1, _2));

        // Initiate our variables 
        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        cartesian_stiffness_.setZero();
        cartesian_damping_.setZero();
        
        return true;
    }

    void FrankController::starting(const ros::Time&) {
        // Our goal with the starting function is to set the target pose / equilibrium point to the current robot state, and calculate the 
        // initial velocities with a jacobian array and set the x_attractor and q_d_nullspace... which I'm not sure what those are still
        // To do this we get the initial robot configuration 
        franka::RobotState initial_state = state_handle_->getRobotState();
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        
        // We're converting our jacobian array to eigen matrices
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        // Need to look into what 'Affine3d' is...
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        // O_T_EE == measured end-effector pose in base frame given in 4x4 matrix, column-major

        // We're... setting the equilibrium point to current state (represented by initial_transform)... from a map of 'initial_state.O_T_EE'... what is O_T_EE?
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
        position_d_target_ = initial_transform.translation();
        orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

        q_d_nullspace_ = q_initial;
    }

    void FrankController::update(const ros::Time&, const ros::Duration&) {
        franka::RobotState robot_state = state_handle_->getRobotState();
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

        // Convert our measured end-effector pose from 4x4 matrix to an Affine3d object called 'transform'
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

        // Extract the position and orientation of the robot end-effector from the 'transform' Affine3d object
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        // Here we are calculating the (position) error from our current position to the desired pose position
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d_;

        // Here we are calculating the orientation error
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }

        // This is our 'difference' quaternion for orientation...  (why do we apply an inverse?)
        // This basically obtains the difference quaternion, converts it to axis angles, and then computes the orientation error
        Eigen::Quaterniond error_quaternioin(orientation * orientation_d_.inverse());
        Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternioin);
        error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

        // Our control variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

        // Need to be more looked into... why are we applying an inverse for 'nullspace handling' ?
        Eigen::MatrixXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        // Here is the meat! This is the Cartesian PD control with damping ratio 1
        tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian*dq));
        
        // What is this 'nullspace' that keeps showing up??
        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -jacobian.transpose() * jacobian_transpose_pinv) * (nullspace_stiffness_ * (q_d_nullspace_ - q) - (2.0 * sqrt(nullspace_stiffness_)) * dq);

        // Desired torque... is this variable tied directly to the hardware-level controllers?
        tau_d << tau_task + tau_nullspace + coriolis;

        // Look into why we saturate the torque rate to avoid discontinuities...
        tau_d << saturateTorqueRate(tau_d, tau_J_d);
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        // I think here we are filtering the input targets and the current state and updating our parameters accordingly
        cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
        cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
        nullspace_stiffness_ = filter_params_* nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;

        // I'm not sure what is going on here...
        // I think what we are doing here is converting the quaternions current orientation 'orientation_d_', target orientation 'orientation_d_target_'
        // and then applying the input filters (and smoothening out the differences?) and then updating our current orientation quaternion 'orientation_d_'
        // again but now with the filters and smoothening out applied to it.
        Eigen::AngleAxisd aa_orientation_d(orientation_d_);
        Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
        aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() + (1.0 - filter_params_) * aa_orientation_d.axis();
        aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() + (1.0 - filter_params_) * aa_orientation_d.angle();
        orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
    }

    Eigen::Matrix<double, 7, 1> FrankController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d) {
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    void FrankController::complianceParamCallback(hafidh_controller::compliance_paramConfig& config, uint32_t) {
        cartesian_stiffness_target_.setIdentity();
        cartesian_stiffness_target_.topLeftCorner(3, 3) << config.translational_stiffness * Eigen::Matrix3d::Identity();
        cartesian_stiffness_target_.bottomRightCorner(3, 3) << config.rotational_stiffness * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.setIdentity();
        cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
        
        nullspace_stiffness_target_ = config.nullspace_stiffness;
    }

    void FrankController::targetPoseHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
        // Here we are actually assigning our Vector3 Eigen 'position_d_target_' with our target message 'msg' of type 'PoseStamped' from 'geometry_msgs'
        position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

        // Here we do the same but with orientation using Quaternions!
        Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
        orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;

        // Not entirely sure what this does... I think it reverses the orientation(?)
        if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
            orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
        }
    }
} // End of our namespace



PLUGINLIB_EXPORT_CLASS(hafidh_controller::FrankController, controller_interface::ControllerBase)
//PLUGINLIB_EXPORT_CLASS(hafidh_controllers::FrankController, controller_interface::ControllerBase)