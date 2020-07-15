/*

    a full re-write because the previous one is FUCKED UP :(  (when calculating velocity)

*/

#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Vector3.h>

const int imu_samples_amount = 20;
const int moving_average = 15;
const int motion_check_samples = 20;
const double motion_check_range = 0.5;

double accel_array_x[imu_samples_amount], accel_array_y[imu_samples_amount], accel_array_z[imu_samples_amount];
double velo_x_curr, velo_x_prev, velo_y_curr, velo_y_prev, velo_z_curr, velo_z_prev;
double time_integral = 0.01;

geometry_msgs::Vector3 filtered_vector;
geometry_msgs::Vector3 accel_vector;
geometry_msgs::Vector3 velocity_vector;
geometry_msgs::Vector3 position_vector;

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector_ref);
void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector_ref, geometry_msgs::Vector3 &filtered_vector_ref);
void processAcceleration(double (&accel_array_x_ref)[imu_samples_amount], double (&accel_array_y_ref)[imu_samples_amount], double (&accel_array_z_ref)[imu_samples_amount], geometry_msgs::Vector3 &filtered_vector_ref, geometry_msgs::Vector3 &accel_vector_ref);
void calculateVelocity(geometry_msgs::Vector3 &velocity_vector_ref, double &time_integral_ref, double (&accel_array_x_ref)[imu_samples_amount], double (&accel_array_y_ref)[imu_samples_amount], double (&accel_array_z_ref)[imu_samples_amount], double &velo_x_curr_ref, double &velo_x_prev_ref, double &velo_y_curr_ref, double &velo_y_prev_ref, double &velo_z_curr_ref, double &velo_z_prev_ref);
void calculatePosition(geometry_msgs::Vector3 &position_vector_ref, double &velo_x_curr_ref, double &velo_x_prev_ref, double &velo_y_curr_ref, double &velo_y_prev_ref, double &velo_z_curr_ref, double &velo_z_prev_ref, double &time_integral_ref);

int main(int argc, char **argv) {
    ros::init(argc, argv, "cartesian_processor");
    ros::NodeHandle cartesian_node;
    ros::Rate loop_rate(100);

    ros::Subscriber cartesian_subscriber = cartesian_node.subscribe("raw_linear_accel", 1000, dataCallback);
    ros::Publisher acceleration_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_accel", 100);
    ros::Publisher velocity_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_vel", 100);
    ros::Publisher position_publisher = cartesian_node.advertise<geometry_msgs::Vector3>("cartesian_pos", 100);

    while (ros::ok()) {
        acceleration_publisher.publish(accel_vector);
        velocity_publisher.publish(velocity_vector);
        position_publisher.publish(position_vector);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

void dataCallback(const geometry_msgs::Vector3::ConstPtr& imu_vector) {
    //ROS_INFO_STREAM("i am getting called...");

    processButter(imu_vector, filtered_vector);
    processAcceleration(accel_array_x, accel_array_y, accel_array_z, filtered_vector, accel_vector);
    //calculateVelocity(velocity_vector, time_integral, accel_array_x, accel_array_y, accel_array_z, velo_x_curr, velo_x_prev, velo_y_curr, velo_y_prev, velo_z_curr, velo_z_prev);
    //calculatePosition(position_vector, velo_x_curr, velo_x_prev, velo_y_curr, velo_y_prev, velo_z_curr, velo_z_prev, time_integral);
    
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

void processButter(const geometry_msgs::Vector3::ConstPtr& imu_vector_ref, geometry_msgs::Vector3 &filtered_vector_ref) {
    static Butter butter_x, butter_y, butter_z;
    filtered_vector_ref.x = butter_x.apply(imu_vector_ref->x);
    filtered_vector_ref.y = butter_y.apply(imu_vector_ref->y);
    filtered_vector_ref.z = butter_z.apply(imu_vector_ref->z);
}

void processAcceleration(double (&accel_array_x_ref)[imu_samples_amount], double (&accel_array_y_ref)[imu_samples_amount], double (&accel_array_z_ref)[imu_samples_amount], geometry_msgs::Vector3 &filtered_vector_ref, geometry_msgs::Vector3 &accel_vector_ref) {
    // Inserting our butterworth-filtered data into our acceleration samples array
    for (int i = imu_samples_amount; i>=1; i--) {
        accel_array_x_ref[i] = accel_array_x_ref[i-1];
        accel_array_y_ref[i] = accel_array_y_ref[i-1];
        accel_array_z_ref[i] = accel_array_z_ref[i-1];
    }
    accel_array_x_ref[0] = filtered_vector_ref.x;   // this will get overriden later, but we need the filtered values so they can be involved in the moving average calculation
    accel_array_y_ref[0] = filtered_vector_ref.y;
    accel_array_z_ref[0] = filtered_vector_ref.z;

    // Now we calculate our final processed (butterworth + moving average) value to our accel_vector
    double array_x_sum, array_y_sum, array_z_sum;
    for (int i = 0; i<moving_average; i++) {
        array_x_sum = array_x_sum + accel_array_x_ref[i];
        array_y_sum = array_y_sum + accel_array_y_ref[i];
        array_z_sum = array_z_sum + accel_array_z_ref[i];
    }
    accel_array_x_ref[0] = array_x_sum / moving_average;
    accel_vector_ref.x = accel_array_x_ref[0];
    accel_array_y_ref[0] = array_y_sum / moving_average;
    accel_vector_ref.y = accel_array_y_ref[0];
    accel_array_z_ref[0] = array_z_sum / moving_average;
}

void calculateVelocity(geometry_msgs::Vector3 &velocity_vector_ref, double &time_integral_ref, double (&accel_array_x_ref)[imu_samples_amount], double (&accel_array_y_ref)[imu_samples_amount], double (&accel_array_z_ref)[imu_samples_amount], double &velo_x_curr_ref, double &velo_x_prev_ref, double &velo_y_curr_ref, double &velo_y_prev_ref, double &velo_z_curr_ref, double &velo_z_prev_ref) {
    int count_x = 0, count_y = 0, count_z = 0;
    double area_x = 0, area_y = 0, area_z = 0;

    for (int i = 0; i<imu_samples_amount; i++) {
        if ((accel_array_x_ref[i] <= motion_check_range) && (accel_array_x_ref[i] >= -motion_check_range)) {
            count_x++;
        };
        if ((accel_array_y_ref[i] <= motion_check_range) && (accel_array_y_ref[i] >= -motion_check_range)) {
            count_y++;
        };
        if ((accel_array_z_ref[i] <= motion_check_range) && (accel_array_y_ref[i] >= -motion_check_range)) {
            count_z++;
        };
    }

    if (count_x >= motion_check_samples) {
        // stopped moving... force velocity to zero
        velo_x_prev_ref = velo_x_curr_ref;
        velo_x_curr_ref = velo_x_curr_ref * 0.15;
    } else {
        // still moving / acceleration values are noisy
        area_x = (time_integral_ref) * ( (accel_array_x_ref[0] + accel_array_x_ref[1]) / 2 );

        velo_x_prev_ref = velo_x_curr_ref;
        velo_x_curr_ref = velo_x_curr_ref + area_x;
    }

    if (count_y >= motion_check_samples) {
        // stopped moving... force velocity to zero
        velo_y_prev_ref = velo_y_curr_ref;
        velo_y_curr_ref = velo_y_curr_ref * 0.15;
    } else {
        // still moving / acceleration values are noisy
        area_y = (time_integral_ref) * ( (accel_array_y_ref[0] + accel_array_y_ref[1]) / 2 );

        velo_y_prev_ref = velo_y_curr_ref;
        velo_y_curr_ref = velo_y_curr_ref + area_y;
    }

    if (count_z >= motion_check_samples) {
        // stopped moving... force velocity to zero
        velo_z_prev_ref = velo_z_curr_ref;
        velo_z_curr_ref = velo_z_curr_ref * 0.15;
    } else {
        // still moving / acceleration values are noisy
        area_z = (time_integral_ref) * ( (accel_array_z_ref[0] + accel_array_z_ref[1]) / 2 );

        velo_z_prev_ref = velo_z_curr_ref;
        velo_z_curr_ref = velo_z_curr_ref + area_x;
    }

    velocity_vector_ref.x = velo_x_curr_ref;
    velocity_vector_ref.y = velo_y_curr_ref;
    velocity_vector_ref.z = velo_z_curr_ref;
}

void calculatePosition(geometry_msgs::Vector3 &position_vector_ref, double &velo_x_curr_ref, double &velo_x_prev_ref, double &velo_y_curr_ref, double &velo_y_prev_ref, double &velo_z_curr_ref, double &velo_z_prev_ref, double &time_integral_ref) {
    double area_x = 0, area_y = 0, area_z = 0;
    area_x = (time_integral_ref) * ( (velo_x_prev_ref + velo_x_curr_ref) / 2);
    area_y = (time_integral_ref) * ( (velo_y_prev_ref + velo_y_curr_ref) / 2);
    area_z = (time_integral_ref) * ( (velo_z_prev_ref + velo_z_curr_ref) / 2);

    position_vector_ref.x = position_vector_ref.x + area_x;
    position_vector_ref.y = position_vector_ref.y + area_y;
    position_vector_ref.z = position_vector_ref.z + area_z;
}
