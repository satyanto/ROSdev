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
#include <geometry_msgs/PoseStamped.h>

const int imu_samples_amount = 25;
const int moving_average_samples = 10;
const int motion_check_samples = 25;

geometry_msgs::Vector3 bf;

// Combined...'d?
double combined_array_x[imu_samples_amount], combined_array_y[imu_samples_amount], combined_array_z[imu_samples_amount];
double *combined_array_x_ptr = combined_array_x;
double *combined_array_y_ptr = combined_array_y;
double *combined_array_z_ptr = combined_array_z;

double combined_x, combined_y, combined_z;
double *combined_x_ptr = &combined_x;
double *combined_y_ptr = &combined_y;
double *combined_z_ptr = &combined_z;

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

ros::Publisher acceleration_publisher;
ros::Publisher velocity_publisher;
ros::Publisher position_publisher;

// Function/Class Declarations
void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector);
void calculateTargetPose(double* position_ptr);
void calculatePosition(double* position_ptr, double* velocity_ptr, double* time_ptr);
void calculateVelocity(double* velocity_ptr, double* accel_ptr, double* time_ptr);
bool stoppedMoving(double* accel_ptr);
void checkMovement(double* x_accel_ptr, double* y_accel_ptr, double* z_accel_ptr, double* x_velo_ptr, double* y_velo_ptr, double* z_velo_ptr);
void processMovingAverage(double* x_array_ptr, double* y_array_ptr, double* z_array_ptr, double x_sample, double y_sample, double z_sample, double &combined_x_ref, double &combined_y_ref, double &combined_z_ref);
void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector, geometry_msgs::Vector3& processed_ref);
void arrayInsertShift(double* x_array_ptr, double* y_array_ptr, double* z_array_ptr, double x_sample, double y_sample, double z_sample);
void processMovingAverageArray(double* x_array_ptr, double* y_array_ptr, double* z_array_ptr, double &combined_x_ref, double &combined_y_ref, double &combined_z_ref);
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

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector) {

    geometry_msgs::Vector3 acceleration_vector;
    geometry_msgs::Vector3 velocity_vector;
    geometry_msgs::Vector3 position_vector;

    // we filter our the data using a butterowrth filter
    processButter(imu_vector, bf);

    processMovingAverage(combined_array_x_ptr, combined_array_y_ptr, combined_array_z_ptr, bf.x, bf.y, bf.z, combined_x, combined_y, combined_z);

    time_stamp[0] = 0.00;
    time_stamp[1] = 0.01;

    // in case a better filter/smoother algorithm is used... set up current acceleration based on 'combined' acceleration values
    *&acc_x_array_ptr[0] = *&acc_x_array_ptr[1];
    *&acc_y_array_ptr[0] = *&acc_y_array_ptr[1];
    *&acc_z_array_ptr[0] = *&acc_z_array_ptr[1];
    *&acc_x_array_ptr[1] = *&combined_array_x_ptr[0];
    *&acc_y_array_ptr[1] = *&combined_array_y_ptr[0];
    *&acc_z_array_ptr[1] = *&combined_array_z_ptr[0];

    // integrate our acceleration values to calculate velocity and throw into the 'vel_..' array
    calculateVelocity(vel_x_array_ptr, acc_x_array_ptr, time_stamp_ptr);
    calculateVelocity(vel_y_array_ptr, acc_y_array_ptr, time_stamp_ptr);
    calculateVelocity(vel_z_array_ptr, acc_z_array_ptr, time_stamp_ptr);

    checkMovement(combined_array_x_ptr, combined_array_y_ptr, combined_array_z_ptr, vel_x_array_ptr, vel_y_array_ptr, vel_z_array_ptr);

    // before we integrate our velocities, we make sure that unequal positive/negative acceleration areas force-zero the velocity values
    // if (stoppedMoving(combined_array_x_ptr)) {
    //     *&vel_x_array_ptr[0] = *&vel_x_array_ptr[1];
    //     *&vel_x_array_ptr[1] = *&vel_x_array_ptr[1]*0.15;
    // }

    // if (stoppedMoving(combined_array_y_ptr)) {
    //     *&vel_y_array_ptr[0] = *&vel_y_array_ptr[1];
    //     *&vel_y_array_ptr[1] = *&vel_y_array_ptr[1]*0.15;
    // }

    // if (stoppedMoving(combined_array_z_ptr)) {
    //     *&vel_z_array_ptr[0] = *&vel_z_array_ptr[1];
    //     *&vel_z_array_ptr[1] = *&vel_z_array_ptr[1]*0.15;
    // }

    calculatePosition(pos_x_array_ptr, vel_x_array_ptr, time_stamp_ptr);
    calculatePosition(pos_y_array_ptr, vel_y_array_ptr, time_stamp_ptr);
    calculatePosition(pos_z_array_ptr, vel_z_array_ptr, time_stamp_ptr);

    acceleration_vector.x = *combined_x_ptr;
    acceleration_vector.y = *combined_y_ptr;
    acceleration_vector.z = *combined_z_ptr;
    velocity_vector.x = *&vel_x_array_ptr[1];
    velocity_vector.y = *&vel_y_array_ptr[1];
    velocity_vector.z = *&vel_z_array_ptr[1];
    position_vector.x = *&pos_x_array_ptr[1];
    position_vector.y = *&pos_y_array_ptr[1];
    position_vector.z = *&pos_z_array_ptr[1];

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

void checkMovement(double* x_accel_ptr, double* y_accel_ptr, double* z_accel_ptr, double* x_velo_ptr, double* y_velo_ptr, double* z_velo_ptr) {
    int x_count = 0, y_count = 0, z_count = 0;
    for (int i=0; i<imu_samples_amount; i++) {
        if ((*&x_accel_ptr[i] <= 1) && (*&x_accel_ptr[i] >= -1)) {
            x_count++;
        }
        if ((*&y_accel_ptr[i] <= 1) && (*&y_accel_ptr[i] >= -1)) {
            y_count++;
        }
        if ((*&z_accel_ptr[i] <= 1) && (*&z_accel_ptr[i] >= -1)) {
            z_count++;
        }
    }
    if (x_count >= motion_check_samples) {
        *&vel_x_array_ptr[0] = *&vel_x_array_ptr[1];
        *&vel_x_array_ptr[1] = *&vel_x_array_ptr[1]*0.15;
    }
    if (y_count >= motion_check_samples) {
        *&vel_y_array_ptr[0] = *&vel_y_array_ptr[1];
        *&vel_y_array_ptr[1] = *&vel_y_array_ptr[1]*0.15;
    }
    if (z_count >= motion_check_samples) {
        *&vel_z_array_ptr[0] = *&vel_z_array_ptr[1];
        *&vel_z_array_ptr[1] = *&vel_z_array_ptr[1]*0.15;
    }
}

bool stoppedMoving(double* accel_ptr) {
    int count;
    for (int i=0; i<imu_samples_amount; i++) {
        if (*&accel_ptr[i] <= 1 && *&accel_ptr[i] >= -1) {
            count++;
        }
    }
    if (count >= motion_check_samples) {
        return true;
    } else {
        return false;
    }
}

void processMovingAverage(double* x_array_ptr, double* y_array_ptr, double* z_array_ptr, double x_sample, double y_sample, double z_sample, double &combined_x_ref, double &combined_y_ref, double &combined_z_ref) {
    arrayInsertShift(x_array_ptr, y_array_ptr, z_array_ptr, x_sample, y_sample, z_sample);
    processMovingAverageArray(x_array_ptr, y_array_ptr, z_array_ptr, combined_x_ref, combined_y_ref, combined_z_ref);
    arrayInsertShift(x_array_ptr, y_array_ptr, z_array_ptr, combined_x_ref, combined_y_ref, combined_z_ref);
}

void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector, geometry_msgs::Vector3 &processed_ref) {
    static Butter butter_x, butter_y, butter_z;
    processed_ref.x = butter_x.apply(imu_vector->x);
    processed_ref.y = butter_y.apply(imu_vector->y);
    processed_ref.z = butter_z.apply(imu_vector->z);
}

void arrayInsertShift(double* x_array_ptr, double* y_array_ptr, double* z_array_ptr, double x_sample, double y_sample, double z_sample) {
    for (int i = imu_samples_amount; i>0; i--) {
        *&x_array_ptr[i] = *&x_array_ptr[i-1];
        *&y_array_ptr[i] = *&y_array_ptr[i-1];
        *&z_array_ptr[i] = *&z_array_ptr[i-1];
    }
    *&x_array_ptr[0] = x_sample;
    *&y_array_ptr[0] = y_sample;
    *&z_array_ptr[0] = z_sample;
}

void processMovingAverageArray(double* x_array_ptr, double* y_array_ptr, double* z_array_ptr, double &combined_x_ref, double &combined_y_ref, double &combined_z_ref) {
    double arraysum_x = 0, arraysum_y = 0, arraysum_z = 0;
    for (int i=0; i<moving_average_samples; i++) {
        arraysum_x = arraysum_x + *&x_array_ptr[i];
        arraysum_y = arraysum_y + *&y_array_ptr[i];
        arraysum_z = arraysum_z + *&z_array_ptr[i];
    }
    combined_x_ref = (arraysum_x)/(moving_average_samples);
    combined_y_ref = (arraysum_y)/(moving_average_samples);
    combined_z_ref = (arraysum_z)/(moving_average_samples);
}

// for testing purposes
void printArray(double* arrayptr) {
    for (int i = 0; i<=imu_samples_amount; i++) {
        std::cout << *&arrayptr[i] << " ";
    }
}