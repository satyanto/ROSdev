#include <ros/ros.h>

#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>



std::vector<double> initialpose = {
    0.306668786697,     // [0] position X
    -0.000106201152198, // [1] position Y
    0.486316449505,     // [2] position Z
    0.999999301298,     // [3] orientation X
    -0.000104675069344, // [4] orientation Y
    -0.00111372893921,  // [5] orientation Z
    0.00038217036592    // [6] orientation W
};

namespace hafidh_controller {
    int main(int argc, char **argv) {
        ros::init(argc, argv, "target_publisher");

        ros::NodeHandle node;
        ros::Publisher target_publisher = node.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
        ros::Rate loop_rate(4);

        // we put this here so... we only make 1 PoseStamped data structure
        geometry_msgs::PoseStamped target_pose;

        int count = 0;
        while (ros::ok()) {
            if (count<=1) {
                target_pose.header.frame_id = "panda_link0";
                target_pose.pose.position.x = initialpose[0];
                target_pose.pose.position.y = initialpose[1];
                target_pose.pose.position.z = initialpose[2];
                target_pose.pose.orientation.x = initialpose[3] ;
                target_pose.pose.orientation.y = initialpose[4];
                target_pose.pose.orientation.z = initialpose[5];
                target_pose.pose.orientation.w = initialpose[6];
                
                // publish our message
                target_publisher.publish(target_pose);

            } else {
                // int randomNumber = rand()%(max-min+1)+min;
                int randX = rand()%(80-0+1) + 0;
                int randY = rand()%(40-0+1) + 0;
                int randZ = rand()%(40-0+1) + 0;

                target_pose.header.frame_id = "panda_link0";
                target_pose.pose.position.x = initialpose[0]+(randX/1000);
                target_pose.pose.position.y = initialpose[1]+(randY/1000);
                target_pose.pose.position.z = initialpose[2]+(randZ/1000);
                target_pose.pose.orientation.x = 0;
                target_pose.pose.orientation.y = 0;
                target_pose.pose.orientation.z = 0;
                target_pose.pose.orientation.w = 1;

                target_publisher.publish(target_pose);
            }

            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }

        return 0;
    }
}