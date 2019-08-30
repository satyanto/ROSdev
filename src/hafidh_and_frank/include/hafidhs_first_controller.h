// Hafidh Satyanto
// I saw in the example controllers that the src .cpp file includes a .h library file... for convenience or necessity NO IDEA.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>            // Controller interface for combining multiple interfaces in one controller instance.
#include <hardware_interface/robot_hw.h>                                // Default ROS hardware interface library?

#include <ros/node_handle.h>                                            // Default ROS Node handle library
#include <ros/time.h>                                                   // Default ROS time library

#include <franka_hw/franka_state_interface.h>                           // Reads the full robot state.
#include <franka_hw/franka_cartesian_command_interface.h>               // Looks like an intermediary handler for Franka cartesian poses and ROS's low-level hardware interface.

namespace hafidh_example_controllers {                                  // Technically unnecessary in a .h library file, but allows us to access our interface class inside hafidh_example_controllers.
    class HafidhsFirstController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaPoseCartesianInterface,
                                        franka_hw::FrankaStateInterface> {
    public:                                                             // The children of the HafidhsFirstController class: 2 functions, starting & update, and 1 bool, init.
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    
    private:
        franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
        ros::Duration elapsed_time_;
        std::array<double, 16> initial_pose_{};
    };
}