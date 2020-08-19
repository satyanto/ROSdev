#include <ros/ros.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Vector3 arkit_position;

void dataCallback(const geometry_msgs::Vector3::ConstPtr& arkit_position);

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

    ros::init(argc, argv, "arkit_publisher");
    ros::NodeHandle node;

    ros::Subscriber cartesian_subscriber = node.subscribe<geometry_msgs::Vector3>("arkit_pos", 100, dataCallback);
    ros::Publisher target_publisher = node.advertise<geometry_msgs::PoseStamped>("/target_pose", 20);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {

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

            target_pose.header.frame_id = "panda_link0";
            target_pose.pose.position.x = initialpose[0]+arkit_position.x;
            target_pose.pose.position.y = initialpose[1]+arkit_position.y;
            target_pose.pose.position.z = initialpose[2]+arkit_position.z;

            target_publisher.publish(target_pose);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

void dataCallback(const geometry_msgs::Vector3::ConstPtr& arkit_vector) {
    double x_upper_limits = 0.25;
    double x_lower_limit = -0.2;

    double y_upper_limits = 0.12;
    double y_lower_limits = -0.10;
    
    double z_upper_limits = 0.175;
    double z_lower_limits = -0.175;

    if (arkit_vector->x <= x_upper_limits && arkit_vector->x >= x_lower_limit) {
        arkit_position.x = arkit_vector->x;
    };
    if (arkit_vector->y <= y_upper_limits && arkit_vector->y >= y_lower_limits) {
        arkit_position.y = arkit_vector->y;
    }; 
    if (arkit_vector->z <= z_upper_limits && arkit_vector->z >= z_lower_limits) {
        arkit_position.z = arkit_vector->z;
    };
}