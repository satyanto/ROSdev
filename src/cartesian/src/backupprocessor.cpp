/* 

    where all the magic happens!
        - filters raw linear acceleration values
        - integrates to velocity (trapezoidal integration)
        - filters velocity values
        - integrates to position
        - filters position

*/



#include <ros/ros.h>
#include <stdio.h>

#include <geometry_msgs/Vector3.h>

// How many samples of the IMU's raw linear acceleration data is taken to smoothen the data
// Does not work at 10 samples -- have a feeling it's because 
const int imu_samples_amount = 40;
const int moving_average_samples = 15;
// Sometimes, the negative side of the acceleration does not equal the area of the positive side
// of the acceleration, leading to the integration never reaching zero.
// The below checks how many samples of acceleration is checked to be 'zero' in order to 'force' a zero velocity
// Must be a number smaller than 'imu_samples_amount' because that is the only acceleration samples kept.
const int motion_check_samples = 40;

// Moving Average'd
//double ma_array_x[imu_samples_amount], ma_array_y[imu_samples_amount], ma_array_z[imu_samples_amount];
//double *ma_array_x_ptr = ma_array_x;
//double *ma_array_y_ptr = ma_array_y;
//double *ma_array_z_ptr = ma_array_z;
//double ma_x, ma_y, ma_z;

// Butterworth Filter'd
geometry_msgs::Vector3 bf;

// Combined...'d?
double combined_array_x[imu_samples_amount], combined_array_y[imu_samples_amount], combined_array_z[imu_samples_amount];
double *combined_array_x_ptr = combined_array_x;
double *combined_array_y_ptr = combined_array_y;
double *combined_array_z_ptr = combined_array_z;
double combined_x, combined_y, combined_z;

// Position'd...?
double pos_x[2], pos_y[2], pos_z[2];    // [1] is current, [0] is previous  
double *pos_x_array_ptr = pos_x;
double *pos_y_array_ptr = pos_y;
double *pos_z_array_ptr = pos_z;
double vel_x[2], vel_y[2], vel_z[2];    // [1] is current, [0] is previous
double *vel_x_array_ptr = vel_x;
double *vel_y_array_ptr = vel_y;
double *vel_z_array_ptr = vel_z;
double acc_x[2], acc_y[2], acc_z[2];    // [1] is current, [0] is previous
double *acc_x_array_ptr = acc_x;
double *acc_y_array_ptr = acc_y;
double *acc_z_array_ptr = acc_z;
double time_stamp[2];                   // [1] is current, [0] is previous
double *time_stamp_ptr = time_stamp;

geometry_msgs::Vector3 acceleration_vector;
geometry_msgs::Vector3 velocity_vector;
geometry_msgs::Vector3 position_vector;

ros::Publisher acceleration_publisher;
ros::Publisher velocity_publisher;
ros::Publisher position_publisher;

// Function/Class Declarations
void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector);
void calculatePosition(double* position_ptr, double* velocity_ptr, double* time_ptr);
void calculateVelocity(double* velocity_ptr, double* accel_ptr, double* time_ptr);
bool stoppedMoving(double* accel_ptr);
void processMovingAverage(double* array_ptr, double imu_sample, double &processed_ref);
void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector, geometry_msgs::Vector3 &processed_ref);
void arrayInsertShift(double* array_ptr, double sample);
void processMovingAverageArray(double* array_ptr, double &processed_ref);
void printArray(double* array_ptr);

class Butter {  // taken from github.com/ZacharyTaylor/butter, thanks!
    public:
        Butter() {

            // with lowpass butter filter, 200 samples, corner1 25 Hz, corner2 0 Hz, logmin -20
            //gain_ = 1.024264069e+01;
            //a_[0] = -0.3333333333;
            //a_[1] = 0.9428090416;

            gain_ = 1.482463775e+01;
            a_[0] = -0.4128015981;
            a_[1] = 1.1429805025;
            initialized = false;
        }

        double apply(double sample) {
            if (!initialized) {
                initialized = true;
                return reset(sample);
            }
            xs_[0] = xs_[1];
            xs_[1] = xs_[2];
            xs_[2] = sample / gain_;
            ys_[0] = ys_[1];
            ys_[1] = ys_[2];
            ys_[2] = (xs_[0] + xs_[2]) + 2*xs_[1] + (a_[0]*ys_[0]) + (a_[1]*ys_[1]);
            return ys_[2];
        }
        double reset(double sample) {
            xs_[0] = sample;
            xs_[1] = sample;
            xs_[2] = sample;
            ys_[0] = sample;
            ys_[1] = sample;
            ys_[2] = sample;
            return sample;
        }
    private:
        bool initialized;
        double a_[2];
        double gain_;
        double xs_[3];
        double ys_[3];
};










int main(int argc, char **argv) {
    ros::init(argc, argv, "cartesian_processor");
    ros::NodeHandle cartesian_node;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Subscriber cartesian_subscriber = cartesian_node.subscribe("raw_linear_accel", 1000, dataCallback);
    //ros::Publisher cartesian_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("ma_accel", 100);
    //ros::Publisher butter_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("buttered_accel", 100);
    acceleration_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_accel", 100);
    velocity_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_velocity", 100);
    position_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_position", 100);
    //ros::Rate loop_rate(100);   // 100 Hz
    ros::waitForShutdown();

}







// int main(int argc, char **argv) {

//     // the actual 'flow' of this program is that the function calls are done through the ROS callback() function which
//     // directly changes our Vector3 variables through pointers.
//     // set up our ROS node along with our publisher topics
//     ros::init(argc, argv, "cartesian_processor");
//     ros::NodeHandle cartesian_node;
//     ros::Subscriber cartesian_subscriber = cartesian_node.subscribe("raw_linear_accel", 1000, dataCallback);
//     //ros::Publisher cartesian_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("ma_accel", 100);
//     //ros::Publisher butter_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("buttered_accel", 100);
//     ros::Publisher acceleration_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_accel", 100);
//     ros::Publisher velocity_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_velocity", 100);
//     ros::Publisher position_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_position", 100);
//     ros::Rate loop_rate(100);   // 100 Hz

//     while(ros::ok()) {
//         //geometry_msgs::Vector3 ma_accel;
//         geometry_msgs::Vector3 acceleration_vector;
//         geometry_msgs::Vector3 velocity_vector;
//         geometry_msgs::Vector3 position_vector;

//         // publish our processed moving average based on raw values
//         //ma_accel.x = ma_x;
//         //ma_accel.y = ma_y;
//         //ma_accel.z = ma_z;
//         //cartesian_publisher.publish(ma_accel);

//         // publish our filtered data
//         //butter_publisher.publish(bf);

//         // publish our combined (butterworth + moving average) values
//         acceleration_vector.x = combined_x;
//         acceleration_vector.y = combined_y;
//         acceleration_vector.z = combined_z;
//         velocity_vector.x = vel_x[1];
//         velocity_vector.y = vel_y[1];
//         velocity_vector.z = vel_z[1];
//         position_vector.x = pos_x[1];
//         position_vector.y = pos_y[1];
//         position_vector.z = pos_z[1];

//         acceleration_publisher.publish(acceleration_vector);
//         velocity_publisher.publish(velocity_vector);
//         position_publisher.publish(position_vector);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     //ros::spin();
//     return 0;
// }

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector) {
    // DEPRECATED
    // we process the moving average of our samples
    //processMovingAverage(ma_array_x_ptr, imu_vector->x, ma_x);
    //processMovingAverage(ma_array_y_ptr, imu_vector->y, ma_y);
    //processMovingAverage(ma_array_z_ptr, imu_vector->z, ma_z);

    // we filter our the data using a butterowrth filter
    processButter(imu_vector, bf);

    // just extra step... we then use the moving average again on the filtered data
    processMovingAverage(combined_array_x_ptr, bf.x, combined_x);
    processMovingAverage(combined_array_y_ptr, bf.y, combined_y);
    processMovingAverage(combined_array_z_ptr, bf.z, combined_z);

    // get the current UNIX Epoch Time and update the previous one
    //time_stamp[0] = time_stamp[1];
    //time_stamp[1] = ros::Time::now().toSec();
    // the above gives very small values for some reason so... they are meant to be 100Hz so I assumed a 0.01 second interval
    time_stamp[0] = 0.00;
    time_stamp[1] = 0.05;

    //std::cout << time_stamp[1] << std::endl;

    // in case a better filter/smoother algorithm is used... set up current acceleration based on 'combined' acceleration values
    acc_x[0] = acc_x[1];
    acc_y[0] = acc_y[1];
    acc_z[0] = acc_z[1];
    acc_x[1] = combined_array_x[0];
    acc_y[1] = combined_array_y[0];
    acc_z[1] = combined_array_z[0];

    // integrate our acceleration values to calculate velocity and throw into the 'vel_..' array
    calculateVelocity(vel_x_array_ptr, acc_x_array_ptr, time_stamp_ptr);
    calculateVelocity(vel_y_array_ptr, acc_y_array_ptr, time_stamp_ptr);
    calculateVelocity(vel_z_array_ptr, acc_z_array_ptr, time_stamp_ptr);

    // before we integrate our velocities, we make sure that unequal positive/negative acceleration areas force-zero the velocity values
    if (stoppedMoving(combined_array_x_ptr)) {
        vel_x[0] = vel_x[1];
        vel_x[1] = vel_x[1]*0.15;
    } else {
        calculatePosition(pos_x_array_ptr, vel_x_array_ptr, time_stamp_ptr);
    }

    if (stoppedMoving(combined_array_y_ptr)) {
        vel_y[0] = vel_y[1];
        vel_y[1] = vel_y[1]*0.15;
    } else {
        calculatePosition(pos_y_array_ptr, vel_y_array_ptr, time_stamp_ptr);
    }

    if (stoppedMoving(combined_array_z_ptr)) {
        vel_z[0] = vel_z[1];
        vel_z[1] = vel_z[1]*0.15;
    } else {
        calculatePosition(pos_z_array_ptr, vel_z_array_ptr, time_stamp_ptr);
    }

    acceleration_vector.x = combined_x;
    acceleration_vector.y = combined_y;
    acceleration_vector.z = combined_z;
    velocity_vector.x = vel_x[1];
    velocity_vector.y = vel_y[1];
    velocity_vector.z = vel_z[1];
    position_vector.x = pos_x[1];
    position_vector.y = pos_y[1];
    position_vector.z = pos_z[1];

    acceleration_publisher.publish(acceleration_vector);
    velocity_publisher.publish(velocity_vector);
    position_publisher.publish(position_vector);
}












void calculatePosition(double* position_ptr, double* velocity_ptr, double* time_ptr) {
    double area = 0;
    area = (*&time_ptr[1] - *&time_ptr[0])*((*&velocity_ptr[0] + *&velocity_ptr[1])/2);
    *&position_ptr[0] = *&position_ptr[1];
    *&position_ptr[1] = *&position_ptr[1] + area;
}

void calculateVelocity(double* velocity_ptr, double* accel_ptr , double* time_ptr) {
    double area = 0;
    area = (*&time_ptr[1] - *&time_ptr[0])*((*&accel_ptr[0] + *&accel_ptr[1])/2);
    *&velocity_ptr[0] = *&velocity_ptr[1];  // shifts back the current velocity to the previous velocity
    *&velocity_ptr[1] = *&velocity_ptr[1] + area;   // add the calculated velocity to the current velocity
}

bool stoppedMoving(double* accel_ptr) {
    int count;
    for (int i=0; i<imu_samples_amount; i++) {
        if (*&accel_ptr[i] <= 0.5 && *&accel_ptr[i] >= -0.5) {
            count++;
        }
    }
    if (count >= motion_check_samples) {
        return true;
    } else {
        return false;
    }
}

void processMovingAverage(double* array_ptr, double imu_sample, double &processed_ref) {
    arrayInsertShift(array_ptr, imu_sample);
    processMovingAverageArray(array_ptr, processed_ref);
    arrayInsertShift(array_ptr, processed_ref);
}

void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector, geometry_msgs::Vector3 &processed_ref) {
    static Butter butter_x, butter_y, butter_z;
    processed_ref.x = butter_x.apply(imu_vector->x);
    processed_ref.y = butter_y.apply(imu_vector->y);
    processed_ref.z = butter_z.apply(imu_vector->z);
}

void arrayInsertShift(double* arrayptr, double sample) {
    for (int i = imu_samples_amount; i>=0; i--) {
        *&arrayptr[i] = *&arrayptr[i-1];
    }
    *&arrayptr[0] = sample;
}

void processMovingAverageArray(double* arrayptr, double &processed_ref) {
    double arraysum;
    for (int i=0; i<moving_average_samples; i++) {
        arraysum = arraysum + *&arrayptr[i];
    }
    processed_ref = (arraysum)/(moving_average_samples);
}

// for testing purposes
void printArray(double* arrayptr) {
    for (int i = 0; i<=imu_samples_amount; i++) {
        std::cout << *&arrayptr[i] << " ";
    }
}