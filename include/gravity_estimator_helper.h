#ifndef GRAVITY_ESTIMATOR_HELPER_H
#define GRAVITY_ESTIMATOR_HELPER_H

#include <sensor_msgs/Imu.h>
#include <cmath>

#include <iostream>


enum Direction{
    XPOS = 0,
    XNEG,
    YPOS,
    YNEG,
    ZPOS,
    ZNEG,
    NONE
};

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3 &a , const geometry_msgs::Vector3 &b){
    geometry_msgs::Vector3 c;

    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;

    return c;
}

geometry_msgs::Vector3 operator-(const geometry_msgs::Vector3 &a , const geometry_msgs::Vector3 &b){
    geometry_msgs::Vector3 c;

    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;

    return c;
}

geometry_msgs::Vector3 operator/(const geometry_msgs::Vector3 &a , const int &k){
    geometry_msgs::Vector3 c;

    c.x = a.x / k;
    c.y = a.y / k;
    c.z = a.z / k;
    
    return c;
}

std::ostream& operator<<(std::ostream &o, geometry_msgs::Vector3 a){
    o << "["<< a.x << " " << a.y << " " << a.z << "]";
    return o;
}


sensor_msgs::Imu calcMeanVariance(std::vector<sensor_msgs::Imu> &rawData){

    const int bucket_size = rawData.size();
    sensor_msgs::Imu avg;
    assert(avg.linear_acceleration.x == 0);
    // Calculate Mean
    for (int i = 0; i<bucket_size; i++){
        avg.linear_acceleration = avg.linear_acceleration + rawData[i].linear_acceleration;
        avg.angular_velocity = avg.angular_velocity + rawData[i].angular_velocity;
    }

    avg.linear_acceleration = avg.linear_acceleration / bucket_size;
    avg.angular_velocity = avg.angular_velocity / bucket_size;

    // Calculate Variance
    assert(avg.linear_acceleration_covariance[0] == 0);
    for (int i = 0; i<bucket_size; i++){

        {
            geometry_msgs::Vector3 diff = rawData[i].linear_acceleration - avg.linear_acceleration;
            
            avg.linear_acceleration_covariance[0] += diff.x * diff.x;
            avg.linear_acceleration_covariance[4] += diff.y * diff.y;
            avg.linear_acceleration_covariance[8] += diff.z * diff.z;
        }
        {
            geometry_msgs::Vector3 diff = rawData[i].angular_velocity - avg.angular_velocity;
            
            avg.angular_velocity_covariance[0] += diff.x * diff.x;
            avg.angular_velocity_covariance[4] += diff.y * diff.y;
            avg.angular_velocity_covariance[8] += diff.z * diff.z;
        }
    }

    avg.linear_acceleration_covariance[0] /= bucket_size;
    avg.linear_acceleration_covariance[4] /= bucket_size;
    avg.linear_acceleration_covariance[8] /= bucket_size;

    avg.angular_velocity_covariance[0] /= bucket_size;
    avg.angular_velocity_covariance[4] /= bucket_size;
    avg.angular_velocity_covariance[8] /= bucket_size;

    return avg;
}

double getAccelVariance(const sensor_msgs::Imu &avg){
    return avg.linear_acceleration_covariance[0] + avg.linear_acceleration_covariance[4] + avg.linear_acceleration_covariance[8];
}

double getGyroVariance(const sensor_msgs::Imu &avg){
    return avg.angular_velocity_covariance[0] + avg.angular_velocity_covariance[4] + avg.angular_velocity_covariance[8];
}

double getAccelMagnitude(const sensor_msgs::Imu &avg){
    double x2 = avg.linear_acceleration.x * avg.linear_acceleration.x;
    double y2 = avg.linear_acceleration.y * avg.linear_acceleration.y;
    double z2 = avg.linear_acceleration.z * avg.linear_acceleration.z;

    return std::sqrt(x2+y2+z2);
}

double getGyroMagnitude(const sensor_msgs::Imu &avg){
    double x2 = avg.angular_velocity.x * avg.angular_velocity.x;
    double y2 = avg.angular_velocity.y * avg.angular_velocity.y;
    double z2 = avg.angular_velocity.z * avg.angular_velocity.z;

    return std::sqrt(x2+y2+z2);
}

struct Bucket{

    sensor_msgs::Imu avg;
    std::vector<sensor_msgs::Imu> rawData;
    Direction direction;

    Bucket(int bucket_size) : bucket_size(bucket_size) {
        rawData.clear();
        rawData.resize(bucket_size);
    }

    bool getIsFull(){return isFull;}

    void push_back(const sensor_msgs::Imu &msg){
        assert(!isFull);
        rawData[index++] = msg;
        if (index == bucket_size){
            isFull = true;
            avg = calcMeanVariance(rawData);
            direction = calcDirection();
        }
    }

    void reset(){
        index = 0;
        isFull = false;
        direction = NONE;
    }

private:
    int bucket_size;
    int index = 0;
    bool isFull = false;
    double accel_variance_threshold = 0.05;
    double gyro_magnitude_threshold = 5.0/180*M_PI;
    double deviation_threshold = 0.85;

    Direction calcDirection(){
        Direction dir = NONE;

        // Gyro detects rotation, then direction is not valid
        if ( getGyroMagnitude(avg) > gyro_magnitude_threshold ){
            // std::cout << "Gyroscope detects rotation = " << getGyroMagnitude(avg) << std::endl;
            return dir;
        }
            

        // If acceleration variance is big, then direction is not valid
        if ( getAccelVariance(avg) > accel_variance_threshold){
            // std::cout << "Acceleration variance is big = " << getAccelVariance(avg) << std::endl;
            return dir;
        }

        // If acceleration is not around gravity, then direction is not valid
        if ( getAccelMagnitude(avg) < 9.8*0.9 || getAccelMagnitude(avg) > 9.8*1.1 ){
            std::cout << "Acceleration Magnitude is not around gravity = " << getAccelMagnitude(avg) << std::endl;
            return dir;
        }
           

        double x2 = avg.linear_acceleration.x * avg.linear_acceleration.x;
        double y2 = avg.linear_acceleration.y * avg.linear_acceleration.y;
        double z2 = avg.linear_acceleration.z * avg.linear_acceleration.z;
        double d2 = deviation_threshold*deviation_threshold;

        if (y2+z2 < d2){  // x-direction
            if (avg.linear_acceleration.x > 0)
                dir = XPOS;
            else
                dir = XNEG;
        }else if (x2+z2 < d2){ // y-direction
            if (avg.linear_acceleration.y > 0)
                dir = YPOS;
            else
                dir = YNEG;
        }else if (x2+y2 < d2){ // z-direction
            if (avg.linear_acceleration.z > 0)
                dir = ZPOS;
            else
                dir = ZNEG;
        }

        return dir;
    }
};

#endif /* GRAVITY_ESTIMATOR_HELPER_H */
