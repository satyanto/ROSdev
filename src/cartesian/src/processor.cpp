/*

    a full re-write because the previous one is FUCKED UP :(  (when calculating velocity)

*/

#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Vector3.h>

const int imu_samples_amount = 100;
const int moving_average = 15;
const int motion_check_samples = 50;
const double motion_check_range = 0.5;

double accel_array_x[imu_samples_amount], accel_array_y[imu_samples_amount], accel_array_z[imu_samples_amount];
double velo_x_curr, velo_x_prev, velo_y_curr, velo_y_prev, velo_z_curr, velo_z_prev;
double time_integral = 0.01;

geometry_msgs::Vector3 filtered_vector;
geometry_msgs::Vector3 accel_vector, velocity_vector, position_vector;
ros::Subscriber cartesian_subscriber;
ros::Publisher acceleration_publisher, velocity_publisher, position_publisher;

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector);
void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector, geometry_msgs::Vector3 &filtered_vector);
void processAcceleration(double (&accel_array_x)[imu_samples_amount], double (&accel_array_y)[imu_samples_amount], double (&accel_array_z)[imu_samples_amount], geometry_msgs::Vector3 &filtered_vector, geometry_msgs::Vector3 &accel_vector);
void calculateVelocity(geometry_msgs::Vector3 &velocity_vector, double &time_integral, double (&accel_array_x)[imu_samples_amount], double (&accel_array_y)[imu_samples_amount], double (&accel_array_z)[imu_samples_amount], double &velo_x_curr, double &velo_x_prev, double &velo_y_curr, double &velo_y_prev, double &velo_z_curr, double &velo_z_prev);
void calculatePosition(geometry_msgs::Vector3 &position_vector, double &velo_x_curr, double &velo_x_prev, double &velo_y_curr, double &velo_y_prev, double &velo_z_curr, double &velo_z_prev, double &time_integral);

int main(int argc, char **argv) {
    ros::init(argc, argv, "cartesian_processor");
    ros::NodeHandle cartesian_node;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    cartesian_subscriber = cartesian_node.subscribe("raw_linear_accel", 1000, dataCallback);
    acceleration_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_accel", 100);
    velocity_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_vel", 100);
    position_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_pose", 100);

    ros::waitForShutdown();

    return 0;
}

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector) {
    processButter(imu_vector, filtered_vector);
    processAcceleration(accel_array_x, accel_array_y, accel_array_z, filtered_vector, accel_vector);
    calculateVelocity(velocity_vector, time_integral, accel_array_x, accel_array_y, accel_array_z, velo_x_curr, velo_x_prev, velo_y_curr, velo_y_prev, velo_z_curr, velo_z_prev);
    calculatePosition(position_vector, velocity_vector, time_integral);
    
    acceleration_publisher.publish(accel_vector);
    velocity_publisher.publish(velocity_publisher);
    position_publisher.publish(position_vector);
}

class Butter {
    public: 
        Butter() {
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

void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector, geometry_msgs::Vector3 &filtered_vector) {
    static Butter butter_x, butter_y, butter_z;
    filtered_vector.x = butter_x.apply(imu_vector->x);
    filtered_vector.y = butter_y.apply(imu_vector->y);
    filtered_vector.z = butter_z.apply(imu_vector->z);
}

void processAcceleration(double (&accel_array_x)[imu_samples_amount], double (&accel_array_y)[imu_samples_amount], double (&accel_array_z)[imu_samples_amount], geometry_msgs::Vector3 &filtered_vector, geometry_msgs::Vector3 &accel_vector) {
    // Inserting our butterworth-filtered data into our acceleration samples array
    for (int i = imu_samples_amount; i>=0; i--) {
        accel_array_x[i] = accel_array_x[i-1];
        accel_array_y[i] = accel_array_y[i-1];
        accel_array_z[i] = accel_array_z[i-1];
    }
    accel_array_x[0] = filtered_vector.x;   // this will get overriden later, but we need the filtered values so they can be involved in the moving average calculation
    accel_array_y[0] = filtered_vector.y;
    accel_array_z[0] = filtered_vector.z;

    // Now we calculate our final processed (butterworth + moving average) value to our accel_vector
    double array_x_sum, array_y_sum, array_z_sum;
    for (int i = 0; i<moving_average; i++) {
        array_x_sum = array_x_sum + accel_array_x[i];
        array_y_sum = array_y_sum + accel_array_y[i];
        array_z_sum = array_z_sum + accel_array_z[i];
    }
    accel_array_x[0] = array_x_sum / moving_average;
    accel_vector.x = accel_array_x[0];
    accel_array_y[0] = array_y_sum / moving_average;
    accel_vector.y = accel_array_y[0];
    accel_array_z[0] = array_z_sum / moving_average;
}

void calculateVelocity(geometry_msgs::Vector3 &velocity_vector, double &time_integral, double (&accel_array_x)[imu_samples_amount], double (&accel_array_y)[imu_samples_amount], double (&accel_array_z)[imu_samples_amount], double &velo_x_curr, double &velo_x_prev, double &velo_y_curr, double &velo_y_prev, double &velo_z_curr, double &velo_z_prev) {
    int count_x = 0, count_y = 0, count_z = 0;
    double area_x = 0, area_y = 0, area_z = 0;

    for (int i = 0; i<imu_samples_amount; i++) {
        if ((accel_array_x[i] <= motion_check_range) && (accel_array_x[i] >= -motion_check_range)) {
            count_x++;
        };
        if ((accel_array_y[i] <= motion_check_range) && (accel_array_y[i] >= -motion_check_range)) {
            count_y++;
        };
        if ((accel_array_z[i] <= motion_check_range) && (accel_array_y[i] >= -motion_check_range)) {
            count_z++;
        };
    }

    if (count_x >= motion_check_samples) {
        // stopped moving... force velocity to zero
        velo_x_prev = velo_x_curr;
        velo_x_curr = velo_x_curr * 0.15;
    } else {
        // still moving / acceleration values are noisy
        area_x = (time_integral) * ( (accel_array_x[0] + accel_array_x[1]) / 2 );

        velo_x_prev = velo_x_curr;
        velo_x_curr = velo_x_curr + area_x;
    }

    if (count_y >= motion_check_samples) {
        // stopped moving... force velocity to zero
        velo_y_prev = velo_y_curr;
        velo_y_curr = velo_y_curr * 0.15;
    } else {
        // still moving / acceleration values are noisy
        area_y = (time_integral) * ( (accel_array_y[0] + accel_array_y[1]) / 2 );

        velo_y_prev = velo_y_curr;
        velo_y_curr = velo_y_curr + area_y;
    }

    if (count_z >= motion_check_samples) {
        // stopped moving... force velocity to zero
        velo_z_prev = velo_z_curr;
        velo_z_curr = velo_z_curr * 0.15;
    } else {
        // still moving / acceleration values are noisy
        area_z = (time_integral) * ( (accel_array_z[0] + accel_array_z[1]) / 2 );

        velo_z_prev = velo_z_curr;
        velo_z_curr = velo_z_curr + area_x;
    }

    velocity_vector.x = velo_x_curr;
    velocity_vector.y = velo_y_curr;
    velocity_vector.z = velo_z_curr;
}

void calculatePosition(geometry_msgs::Vector3 &position_vector, double &velo_x_curr, double &velo_x_prev, double &velo_y_curr, double &velo_y_prev, double &velo_z_curr, double &velo_z_prev, double &time_integral) {
    double area_x = 0, area_y = 0, area_z = 0;
    area_x = (time_integral) * ( (velo_x_prev + velo_x_curr) / 2);
    area_y = (time_integral) * ( (velo_y_prev + velo_y_curr) / 2);
    area_z = (time_integral) * ( (velo_z_prev + velo_z_curr) / 2);

    position_vector.x = position_vector.x + area_x;
    position_vector.y = position_vector.y + area_y;
    position_vector.z = position_vector.z + area_z;
}
