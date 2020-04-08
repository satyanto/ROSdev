/* 

    where all the magic happens!
        - filters raw linear acceleration values
        - integrates to velocity
        - filters velocity values
        - integrates to position
        - filters position

*/



#include <ros/ros.h>
#include <stdio.h>

#include <geometry_msgs/Vector3.h>

// How many samples of the IMU's raw linear acceleration data is taken to smoothen the data
const int imu_samples_amount = 10;

// Create an array that will be used for a moving average
float imu_samples_x[imu_samples_amount], imu_samples_y[imu_samples_amount], imu_samples_z[imu_samples_amount];
float *array_x_ptr = imu_samples_x;
float *array_y_ptr = imu_samples_y;
float *array_z_ptr = imu_samples_z;

// Process a new 'smoothened' value based on the moving average array
float imu_processed_x, imu_processed_y, imu_processed_z;

// float* processed_x_ptr = &imu_processed_x;
// float* processed_y_ptr = &imu_processed_y;
// float* processed_z_ptr = &imu_processed_z;
// std::array<float, imu_samples_amount> imu_samples_x;
// std::array<float, imu_samples_amount> imu_samples_y;
// std::array<float, imu_samples_amount> imu_samples_z;

// Function Declarations
void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector);
void arrayInsertShift(float* array_ptr, float sample);
void printArray(float* array_ptr);
void processArray(float* array_ptr, float &processed_ptr);



int main(int argc, char **argv) {

    ros::init(argc, argv, "cartesian_processor");
    ros::NodeHandle cartesian_node;
    ros::Subscriber cartesian_subscriber = cartesian_node.subscribe("raw_linear_accel", 1000, dataCallback);
    ros::Publisher cartesian_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("smoothed_linear_accel", 100);
    ros::Rate loop_rate(100);

    while(ros::ok()) {
        geometry_msgs::Vector3 smoothened_linear_accel;

        smoothened_linear_accel.x = imu_processed_x;
        smoothened_linear_accel.y = imu_processed_y;
        smoothened_linear_accel.z = imu_processed_z;

        cartesian_publisher.publish(smoothened_linear_accel);

        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();
    return 0;
}



void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector) {
    //ROS_INFO("imuVector's X: [%s]", std::to_string(imu_vector->x));
    //ROS_INFO("imuVector's X: [%f]", imu_vector->x);
    //std::cout << imu_vector->x << std::endl;
    arrayInsertShift(array_x_ptr, imu_vector->x);
    arrayInsertShift(array_y_ptr, imu_vector->y);
    arrayInsertShift(array_z_ptr, imu_vector->z);
    processArray(array_x_ptr, imu_processed_x);
    processArray(array_y_ptr, imu_processed_y);
    processArray(array_z_ptr, imu_processed_z);
    arrayInsertShift(array_x_ptr, imu_processed_x);
    arrayInsertShift(array_y_ptr, imu_processed_y);
    arrayInsertShift(array_z_ptr, imu_processed_z);
}

void arrayInsertShift(float* arrayptr, float sample) {
    //std::cout << sample << std::endl;
    for (int i = imu_samples_amount; i>=0; i--) {
        *&arrayptr[i] = *&arrayptr[i-1];
        //std::cout << "iteration: " << i << std::endl;
        //*&arrayptr[i] = *&arrayptr[i-1];
    }
    *&arrayptr[0] = sample;
}

void processArray(float* arrayptr, float &processed_ref) {
    float arraysum;
    for (int i=0; i<=imu_samples_amount; i++) {
        arraysum = arraysum + *&arrayptr[i];
    }
    processed_ref = (arraysum)/(imu_samples_amount);
}

void printArray(float* arrayptr) {
    for (int i = 0; i<=imu_samples_amount; i++) {
        //ROS_INFO("[%f] ", array[i]);
        std::cout << *&arrayptr[i] << " ";
    }
    //std::cout << std::endl;
}