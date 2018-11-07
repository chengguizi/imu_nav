/**
 * @file dead_reckoning_demo.cpp
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-21
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <iostream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "dead_reckoning.h"

#include "velocity_observer.h"

#include <std_msgs/Header.h>

#include <cmath>

#include <fstream>
#include <sstream>

ros::Subscriber _imu_sub;
ros::Publisher _odometry_pub;
ros::Publisher _imu_world_pub;

ros::Subscriber _reset_sub;

struct Calib{
        Eigen::Vector3d scale;
        Eigen::Vector3d bias;
        Eigen::Quaternion<double> q_sw; // sensor's world frame in global world frame (ENU)
}imu_calib;

DeadReckoning deadReckoning;
VelocityObserver velocityObserver;

void resetCallback(const std_msgs::HeaderPtr &header){

    std::cout << "Reset occurs at " << header->stamp << std::endl;
    deadReckoning.reset();
}



std::ostringstream streamout;

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu){


    sensor_msgs::Imu::Ptr tr_imu(new sensor_msgs::Imu(*imu));

    // Convert IMU Sensor Coordinate System to standard East-North-Up World Coordinate System
    {
        Eigen::Quaternion<double> q(tr_imu->orientation.w, tr_imu->orientation.x, tr_imu->orientation.y, tr_imu->orientation.z);

        q = imu_calib.q_sw * q; // now q represent transformation of frame from imu-frame to world frame
        q.normalize();

        // We could minus off the BIAS here
        Eigen::Vector3d accel_imu = {tr_imu->linear_acceleration.x, tr_imu->linear_acceleration.y, tr_imu->linear_acceleration.z};
        
        
         
        // std::cout << "bias in world = " << bias_world.transpose() << std::endl;
        Eigen::Vector3d accel_world;
        deadReckoning.transformAcceleration(accel_imu, q, accel_world);

        // static Eigen::Vector3d filtered_accel_world = {0,0,0};



        // As we already rotate the accel readings to global world frame, set q in the msg to identity
        tr_imu->orientation.w = 1;
        tr_imu->orientation.x = 0;
        tr_imu->orientation.y = 0;
        tr_imu->orientation.z = 0;

        // Update the gravity-offset acceleration
        tr_imu->linear_acceleration.x = accel_world(0);
        tr_imu->linear_acceleration.y = accel_world(1);
        tr_imu->linear_acceleration.z = accel_world(2);
        
    }

    velocityObserver.push_back(tr_imu);

    sensor_msgs::Imu::ConstPtr curr_imu_ptr;
    double delta_t;
    VelocityObserver::State vo_state;
    velocityObserver.pop_current(curr_imu_ptr, delta_t, vo_state);

    // Sanity check
    if (curr_imu_ptr == nullptr)
        return;

    assert(delta_t > 0);

    std::string state_str;
    switch(vo_state.state){
        case VelocityObserver::GENERAL_MOTION:
            deadReckoning.setIntegrationMode(DeadReckoning::GENERAL_MOTION); // 0
            state_str = "GENERAL_MOTION";
            break;
        case VelocityObserver::STATIONARY:
            deadReckoning.setIntegrationMode(DeadReckoning::STATIONARY); // 1
            state_str = "STATIONARY";
            break;
        case VelocityObserver::ACCEL_SMALL:
            deadReckoning.setIntegrationMode(DeadReckoning::ACCEL_SMALL); // 2
            state_str = "ACCEL_SMALL";
            break;
        case VelocityObserver::DOMINANT_DIRECTION:
            deadReckoning.setIntegrationMode(DeadReckoning::DOMINANT_DIRECTION); // 3
            state_str = "DOMINANT_DIRECTION";
            break;
        default:
            state_str = "ERROR";
            std::cerr << "Uncaught vo_state" << std::endl;
            break;
    }

    // Debug
    // if (vo_state.state != VelocityObserver::STATIONARY && vo_state.state != VelocityObserver::ACCEL_SMALL)
    //     deadReckoning.setIntegrationMode(DeadReckoning::GENERAL_MOTION);

    std::cout << "State = " << state_str << std::endl;

    if (vo_state.state == VelocityObserver::DOMINANT_DIRECTION){

        double &x = vo_state.dominant_velocity(0);
        double &y = vo_state.dominant_velocity(1);
        double &z = vo_state.dominant_velocity(2);

        double heading = std::atan2(y,x)*180.0/M_PI;
        double horizontal_vel = std::sqrt(x*x+y*y);
        double elevation = std::atan2(z,horizontal_vel)*180.0/M_PI;

        double magnitude = vo_state.dominant_velocity.norm();
        std::cout << std::setprecision(3) << std::setw(8) << std::fixed;
        std::cout << " dominant_velocity = " << magnitude << " (h,e) = "<< heading << " , " << elevation << std::endl;
        std::cout << std::endl;
    }
    

    Eigen::Vector3d accel_world = {curr_imu_ptr->linear_acceleration.x, curr_imu_ptr->linear_acceleration.y, curr_imu_ptr->linear_acceleration.z};


    deadReckoning.process(accel_world, curr_imu_ptr->header.stamp, delta_t, vo_state.dominant_velocity);
    
    nav_msgs::Odometry odometry_msg = deadReckoning.getOdometryMsg();
    sensor_msgs::Imu imu_msg = deadReckoning.getAccelWorldMsg();

    _odometry_pub.publish(odometry_msg);
    _imu_world_pub.publish(imu_msg);

    streamout << curr_imu_ptr->linear_acceleration.y << " " << odometry_msg.twist.twist.linear.y << std::endl;


    
}

int main (int argc, char** argv){
    std::ofstream fout;
    fout.open("accel_world.txt");

    //// ROS
    ros::init(argc, argv, "dead_reckoning_demo");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    bool apply_calibration;
    ROS_ASSERT(local_nh.getParam("apply_calibration", apply_calibration));
    if (apply_calibration){
        std::string imu_calib_file;
        ROS_ASSERT(local_nh.getParam("imu_calib_file", imu_calib_file));

        std::cout << "\n//////////////////Applying Calibration from " << imu_calib_file << std::endl;
        YAML::Node node = YAML::LoadFile(imu_calib_file);

        assert(node["scale"].IsSequence() && node["scale"].size() == 3);

        for (int i=0;i<3;i++)
        {
            imu_calib.scale(i) = node["scale"][i].as<double>();
            std::cout << "scale[" << i << "]=" << imu_calib.scale(i) << std::endl;
        }

        for (int i=0;i<3;i++)
        {
            imu_calib.bias(i) = node["bias"][i].as<double>();
            std::cout << "bias[" << i << "]=" << imu_calib.bias(i)<< std::endl;
        }  
            
    }else
        std::cout << "\n//////////////////Not using calibration file.//////////////////\n" << std::endl;


    // Convert coordinate system if necessary
    std::string imu_topic, imu_coordinate_system;
    ROS_ASSERT(local_nh.getParam("imu_coordinate_system", imu_coordinate_system));

    if (imu_coordinate_system == "ENU"){
        imu_calib.q_sw = Eigen::Quaternion<double>::Identity();

        ROS_WARN("Using ENU Frame (IMU is also ENU frame)");

    }
    else if (imu_coordinate_system == "NED"){
        Eigen::Matrix3d R_sw;

        R_sw << 0 , 1 , 0,
                1 , 0 , 0,
                0 , 0 , -1;

        imu_calib.q_sw = Eigen::Quaternion<double>(R_sw);

        ROS_WARN( "Using ENU Frame (IMU is in NED frame)");
    }
    else{
        ROS_ERROR("Unkown IMU Coordinate Frame: Choose ENU or NED");
    }
    std::cout << "q_sw = " << imu_calib.q_sw.w()  << ", " << imu_calib.q_sw.vec().transpose() << std::endl;

    // Set parameters
    DeadReckoning::Parameters params;
    params.g_project.affine() << imu_calib.scale(0), 0 , 0, imu_calib.bias(0),
                        0 , imu_calib.scale(1), 0 , imu_calib.bias(1),
                        0 , 0 , imu_calib.scale(2) , imu_calib.bias(2);

    deadReckoning.setParams(params);

    // Publishers
    _odometry_pub = nh.advertise<nav_msgs::Odometry>("dead_reckoning",100);
    _imu_world_pub = nh.advertise<sensor_msgs::Imu>("imu_world",100);

    // Subscribe to IMU topic
    ROS_ASSERT(local_nh.getParam("imu_topic", imu_topic));
    _imu_sub = nh.subscribe(imu_topic, 100, &imuCallback);

    // Subscribe to reset topic
    _reset_sub = nh.subscribe("/reset", 1, &resetCallback);

  
    

    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    // ros::waitForShutdown();


    std::cout << "First thing first, describe your current test setup: " << std::endl;
    std::string description;
    std::getline(std::cin, description);

    ros::spin();


    // Prevent Boost mutex error
    _imu_sub.shutdown();
    _odometry_pub.shutdown();
    _reset_sub.shutdown();
    _imu_world_pub.shutdown();


    
    std::cout << "Writing to file..." << std::endl;
    fout << description << std::endl << streamout.str();
    fout.close();

    std::cout << "Done." << std::endl;
}