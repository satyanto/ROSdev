#include <ros/ros.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Vector3 cartesian_velocity;
void dataCallback(const geometry_msgs::Vector3::ConstPtr& cartesian_vector);

int main(int argc, char **argv) {

    std::vector<double> initialpose = {
        0.306668786697,     // [0] position X
        -0.000106201152198, // [1] position Y
        0.486316449505,     // [2] position Z
        0.999999301298,     // [3] orientation X
        -0.000104675069344, // [4] orientation Y
        -0.00111372893921,  // [5] orientation Z
        0.00038217036592    // [6] orientation W
    };

    ros::init(argc, argv, "target_publisher");
    ros::NodeHandle node;

    ros::Subscriber cartesian_subscriber = node.subscribe<geometry_msgs::Vector3>("cartesian_velocity", 100, dataCallback);
    ros::Publisher target_publisher = node.advertise<geometry_msgs::PoseStamped>("/target_pose", 20);
    ros::Rate loop_rate(20);

    // we put this here so... we only make 1 PoseStamped data structure
    //geometry_msgs::PoseStamped target_pose;

    int count = 0;
    while (ros::ok()) {

        //geometry_msgs::PoseStamped target_pose;

        std::cout << count << std::endl;
        if (count<=2) {             // we use this so we don't run into initial pose problems (discontinuous motion) - for the first few seconds, let the robot stay where it is
            geometry_msgs::PoseStamped target_pose;

            std::cout << "initial count" << std::endl;
            target_pose.header.frame_id = "panda_link0";
            target_pose.pose.position.x = initialpose[0];
            target_pose.pose.position.y = initialpose[1];
            target_pose.pose.position.z = initialpose[2];
            //target_pose.pose.orientation.x = initialpose[3] ;
            //target_pose.pose.orientation.y = initialpose[4];
            //target_pose.pose.orientation.z = initialpose[5];
            //target_pose.pose.orientation.w = initialpose[6];
            
            // publish our message
            target_publisher.publish(target_pose);

        } else {
            geometry_msgs::PoseStamped target_pose;
            double x_limits = initialpose[0] + 0.225;
            double y_limits = initialpose[1] + 0.11;
            double z_limits = initialpose[2] + 0.11;

            // int randomNumber = rand()%(max-min+1)+min;

            // double randX = rand()%(40-0+1) + 0;
            // double randY = rand()%(20-0+1) + 0;
            // double randZ = rand()%(20-0+1) + 0;

            // std::cout << randX << ": randX raw,    ";
            // std::cout << randX/150 << ": randX" << std::endl;
            // std::cout << randY << ": randY raw,    ";
            // std::cout << randY/150 << ": randY" << std::endl;
            // std::cout << randZ << ": randZ raw,    ";
            // std::cout << randZ/150 << ": randZ" << std::endl;

            // target_pose.header.frame_id = "panda_link0";
            // target_pose.pose.position.x = initialpose[0]+(randX/150);
            // target_pose.pose.position.y = initialpose[1]+(randY/150);
            // target_pose.pose.position.z = initialpose[2]+(randZ/150);
            // //target_pose.pose.orientation.x = 0;
            // //target_pose.pose.orientation.y = 0;
            // //target_pose.pose.orientation.z = 0;
            // //target_pose.pose.orientation.w = 1;

            // target_pose.header.frame_id = "panda_link0";
            // target_pose.pose.position.x = initialpose[0]+cartesian_position.x;
            // target_pose.pose.position.y = initialpose[1]+cartesian_position.y;
            // target_pose.pose.position.z = initialpose[2]+cartesian_position.z;

            double x_movement = target_pose.pose.position.x + cartesian_velocity.x;
            double y_movement = target_pose.pose.position.y + cartesian_velocity.y;
            double z_movement = target_pose.pose.position.z + cartesian_velocity.z;

            target_pose.header.frame_id = "panda_link0";
            if ((x_movement <= x_limits) && (x_movement >= -x_limits)) {
                target_pose.pose.position.x = x_movement;
            } else {
                target_pose.pose.position.x = target_pose.pose.position.x;
            }
            if ((y_movement <= y_limits) && (y_movement >= -y_limits)) {
                target_pose.pose.position.y = target_pose.pose.position.y + cartesian_velocity.y;
            } else {
                target_pose.pose.position.y = target_pose.pose.position.y;
            }
            if ((z_movement <= z_limits) && (z_movement >= -z_limits)) {
                target_pose.pose.position.z = target_pose.pose.position.z + cartesian_velocity.z;
            } else {
                target_pose.pose.position.z = target_pose.pose.position.z;
            }

            target_publisher.publish(target_pose);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

void dataCallback(const geometry_msgs::Vector3::ConstPtr& cartesian_vector) {
    double reduction_factor = 35;
    cartesian_velocity.x = (cartesian_vector->x)/reduction_factor;
    cartesian_velocity.y = (cartesian_vector->y)/reduction_factor;
    cartesian_velocity.z = (cartesian_vector->z)/reduction_factor;

    // if (cartesian_vector->x <= x_limits && cartesian_vector->x >= -x_limits) {
    //     cartesian_velocity.x = cartesian_vector->x;
    // };
    // if (cartesian_vector->y <= y_limits && cartesian_vector->y >= -y_limits) {
    //     cartesian_velocity.y = cartesian_vector->y;
    // }; 
    // if (cartesian_vector->z <= x_limits && cartesian_vector->z >= -z_limits) {
    //     cartesian_velocity.z = cartesian_vector->z;
    // };
}