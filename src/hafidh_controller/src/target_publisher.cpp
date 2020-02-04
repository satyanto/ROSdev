#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "targetpublishertest");

    ros::NodeHandle node;
    ros::Publisher target_publisher = node.advertise<geometry

}