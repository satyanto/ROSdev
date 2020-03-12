#include <ros/ros.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>



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
    ros::Publisher target_publisher = node.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    ros::Rate loop_rate(1);

    // we put this here so... we only make 1 PoseStamped data structure
    //geometry_msgs::PoseStamped target_pose;

    int count = 0;
    while (ros::ok()) {

        //geometry_msgs::PoseStamped target_pose;

        std::cout << count << std::endl;
        if (count<=2) {
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

            // int randomNumber = rand()%(max-min+1)+min;
            double randX = rand()%(40-0+1) + 0;
            double randY = rand()%(20-0+1) + 0;
            double randZ = rand()%(20-0+1) + 0;

            std::cout << randX/250 << ": randX" << std::endl;
            std::cout << randY/250 << ": randY" << std::endl;
            std::cout << randZ/250 << ": randZ" << std::endl;

            target_pose.header.frame_id = "panda_link0";
            target_pose.pose.position.x = initialpose[0]+(randX/250);
            target_pose.pose.position.y = initialpose[1]+(randY/250);
            target_pose.pose.position.z = initialpose[2]+(randZ/250);
            //target_pose.pose.orientation.x = 0;
            //target_pose.pose.orientation.y = 0;
            //target_pose.pose.orientation.z = 0;
            //target_pose.pose.orientation.w = 1;

            target_publisher.publish(target_pose);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}