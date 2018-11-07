/**
 * @file offline_calibrate.cpp
 * @author Huimin Cheng (NUS)
 * @brief Automatic routine to determine the max/min for each axis, along with it bias
 * @version 0.1
 * @date 2018-10-18
 * 
 * @copyright Copyright (c) 2018
 * 
 */


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <cmath>
#include <fstream>
#include <ctime>

#include <unistd.h>
#include <linux/limits.h>
#include <bitset>

#include "gravity_estimator.h"

bool isCalibrating = false;

const int bucket_size = 16;
const int buffer_size = 64;

GravityEstimator gravityEstimator(bucket_size,buffer_size);

struct Range{
    int accel;
    int gyro;
    const double g = 9.8;
    const int k = 32768;
}imu_range;

double instantaneous_gravity = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu){
    // std::cout << "Received: " << imu->header.seq << std::endl;

    

    if (!isCalibrating){
        if (!instantaneous_gravity)
            instantaneous_gravity = getAccelMagnitude(*imu);
        else
            instantaneous_gravity = instantaneous_gravity*0.99 + 0.01*getAccelMagnitude(*imu);
        std::cout  << "\r" << "Gravity: " << instantaneous_gravity << std::flush;
        return;
    }

    gravityEstimator.push_back(*imu);

}

std::string get_working_path()
{
   char temp[PATH_MAX];
   return ( getcwd(temp, PATH_MAX) ? std::string( temp ) : std::string("") );
}

void get_date_time_dash(std::string &date_time){
    std::time_t result = std::time(nullptr);
    struct tm * timeinfo = localtime(&result);

    char buffer_date[80];
    strftime(buffer_date,80,"imu_calib-%F-%H-%M-%S.yaml",timeinfo);
    date_time = std::string(buffer_date);
}

int main (int argc, char** argv){
    std::cout << "\nAccelerometer Scale and Bias Calibration Utility" << std::endl;
    auto cwd = get_working_path();
    std::cout << "Current Working Directory: " << cwd << std::endl;

    //// ROS
    ros::init(argc, argv, "offline_calibration");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    ros::Subscriber _imu_sub;

    //// Get IMU range, must be consistent with IMU register settings!
    ROS_ASSERT(local_nh.getParam("accel_range", imu_range.accel));
    ROS_ASSERT(local_nh.getParam("gyro_range", imu_range.gyro));

    
    _imu_sub = nh.subscribe("/imu0", 5, &imuCallback);
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    std::cout << "\nPress [Enter] to start calibration..." << std::endl;
    std::cin.get();
    std::cout << "========================Started==========================" << std::endl;
    isCalibrating = true;
   

    std::cout << "Listening to topic " << "/imu0" << std::endl;

    std::cout << "\nPress [Enter] to terminate calibration..." << std::endl;
    std::cin.get();
    isCalibrating = false;
    spinner.stop();
    ros::Duration(0.5).sleep();

    if (gravityEstimator.isDone()){
        std::cout << "========================Completed==========================" << std::endl;
        std::cout << "\n\n\nFinal Accel Limit X: [" << gravityEstimator.xMin << " " << gravityEstimator.xMax << "]"<< std::endl;
        std::cout << " @ Attitude min: " << gravityEstimator.limits[XNEG].linear_acceleration << std::endl;
        std::cout << " @ Attitude max: " << gravityEstimator.limits[XPOS].linear_acceleration << std::endl;

        std::cout << "Final Accel Limit Y: [" << gravityEstimator.yMin << " " << gravityEstimator.yMax << "]" << std::endl;
        std::cout << " @ Attitude min: " << gravityEstimator.limits[YNEG].linear_acceleration << std::endl;
        std::cout << " @ Attitude max: " << gravityEstimator.limits[YPOS].linear_acceleration << std::endl;

        std::cout << "Final Accel Limit Z: [" << gravityEstimator.zMin << " " << gravityEstimator.zMax << "]" << std::endl;
        std::cout << " @ Attitude min: " << gravityEstimator.limits[ZNEG].linear_acceleration << std::endl;
        std::cout << " @ Attitude max: " << gravityEstimator.limits[ZPOS].linear_acceleration << std::endl;


        // The bias is the difference between the measured gravity and the reference
        double bias_x = (gravityEstimator.xMax + gravityEstimator.xMin)/2;
        double bias_y = (gravityEstimator.yMax + gravityEstimator.yMin)/2;
        double bias_z = (gravityEstimator.zMax + gravityEstimator.zMin)/2;


        // The scale factor is the ratio between measured gravity and the reference 9.8 m/s^2
        double scale_x = (gravityEstimator.xMax - gravityEstimator.xMin)/(9.8*2);
        double scale_y = (gravityEstimator.yMax - gravityEstimator.yMin)/(9.8*2);
        double scale_z = (gravityEstimator.zMax - gravityEstimator.zMin)/(9.8*2);

        assert ( scale_x>0 && scale_y>0 && scale_z>0);

        


        // YAML Output
        YAML::Emitter out;
        std::string temp;

        out << YAML::BeginMap;
        out << YAML::Key << "scale" << YAML::Comment("The scale factor is the ratio between measured gravity and the reference 9.8 m/s^2");
        out << YAML::Value << YAML::BeginSeq << scale_x << YAML::Comment("x") << scale_y << YAML::Comment("y") << scale_z << YAML::Comment("z") << YAML::EndSeq;
        out << YAML::Key << "bias" << YAML::Comment("The bias is the difference between the measured gravity and the reference");

        temp = "x: " + std::to_string(bias_x / imu_range.g / imu_range.accel * imu_range.k);
        out << YAML::Value << YAML::BeginSeq << bias_x << YAML::Comment(temp);
        temp = "y: " + std::to_string(bias_y / imu_range.g / imu_range.accel * imu_range.k);
        out << bias_y << YAML::Comment(temp);
        temp = "z: " + std::to_string(bias_z / imu_range.g / imu_range.accel * imu_range.k);
        out << bias_z << YAML::Comment(temp) << YAML::EndSeq;
        out << YAML::EndMap;

        
        temp = "x-limits: [" + std::to_string(gravityEstimator.xMin) + ", " + std::to_string(gravityEstimator.xMax) + "]";
        out << YAML::Newline << YAML::Comment(temp);
        temp = "y-limits: [" + std::to_string(gravityEstimator.yMin) + ", " + std::to_string(gravityEstimator.yMax) + "]";
        out << YAML::Newline << YAML::Comment(temp);
        temp = "z-limits: [" + std::to_string(gravityEstimator.zMin) + ", " + std::to_string(gravityEstimator.zMax) + "]";
        out << YAML::Newline << YAML::Comment(temp);

        

        std::cout << out.c_str() << std::endl;
        
        std::ofstream fout;
        std::string filename;
        get_date_time_dash(filename);
        fout.open(filename);
        fout << out.c_str();
        fout.close();
        
    }else{
        std::cout << "Calibration Incomplete!\n" << std::endl;
    }

    ros::shutdown();
    return 0;
}