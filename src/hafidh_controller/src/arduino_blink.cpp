// This node will publish a message on 'toggle_led' - which will be received by the Arduino
#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {

    ros:: init(argc, argv, "arduino_blinker");

    ros::NodeHandle node;
    ros::Publisher blinker_publisher = node.advertise<std_msgs::Empty>("toggle_led", 8);
    ros::Rate loop_rate(1); // in Hz, so 1/1 is 1 second.

    int count = 0;
    
    while (ros::ok()) {
        std_msgs::Empty toggle;  // remember that std_msgs::Bool is not the same as bool
        blinker_publisher.publish(toggle);  // the arduino is not necessarily looking at the value; it flips on callback.

        loop_rate.sleep();        
    }

    return 0;
}