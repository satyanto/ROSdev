/*

    a full re-write because the previous one is FUCKED UP :(  (when calculating velocity)

*/

#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Vector3.h>

const int imu_samples_amount = 100;
const int moving_average = 15;
const int motion_check_samples = 50;

double accel_array_x[imu_samples_amount], accel_array_y[imu_samples_amount], accel_array_z[imu_samples_amount];
double *accel_array_x_ptr = accel_array_x;
double *accel_array_y_ptr = accel_array_y;
double *accel_array_z_ptr = accel_array_z;

geometry_msgs::Vector3 butterfilter;
geometry_msgs::Vector3 accel_vector, velocity_vector, position_vector;
geometry_msgs::Vector3::ConstPtr accel_vector_ptr, velocity_vector_ptr, position_vector_ptr;

double pos_x_curr, pos_x_prev;
double pos_y_curr, pos_y_prev;
double pos_z_curr, pos_z_prev;
double timeIntegral = 0.01;

ros::Publisher acceleration_publisher, velocity_publisher, position_publisher;

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector);
void processMovingAverage(double* array_x_ptr, double* array_y_ptr, double* array_z_ptr, const geometry_msgs::Vector3::ConstPtr& imu_vector, double*  processed_ref);

