/* Hafidh Satyanto */
// Here's to those who name stuff. Yeet! Frank! Kip!

#include <hafidh_controller/frank_controller.h>

#include <cmath>
#include <memory>

#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>

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
            // (DONE) RENAME ARM_ID TO "PANDA"
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
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_model"));
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
        dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
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

    }

    void FrankController::update(const ros::Time&, const ros::Duration&) {

    }

    Eigen::Matrix<double, 7, 1> FrankController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d) {
        
    }

    void FrankController::complianceParamCallback(franka_example_controllers::compliance_paramConfig& config, uint32_t) {

    }

    void FrankController::targetPoseHandler(const geometry_msgs::PoseStampedConstPtr& msg) {

    }
}