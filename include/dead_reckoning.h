#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H
/**
 * @file dead_reckoning.h
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-21
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
// #include <Eigen/Geometry> 

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <cmath>

#include <ros/ros.h>

class DeadReckoning{
public:
    struct Parameters{
        Eigen::Affine3d g_project; // transformation from world-scale to imu-scale, in imu-coordinates
        Eigen::Vector3d g = {0, 0, 9.8}; // East-North-Up
        // double hp_rc_time = 5;
    };

    enum IntegrationMode{
        GENERAL_MOTION,
        STATIONARY,
        ACCEL_SMALL,
        DOMINANT_DIRECTION
    };

    struct State{ // This is in ENU Frame
        IntegrationMode integration_mode;

        Eigen::Vector3d velocity = {0,0,0};
        Eigen::Vector3d position = {0,0,0};
        Eigen::Vector3d accel = {0,0,0};
        ros::Time stamp = ros::Time(0);

        // Eigen::Vector3d accel_bias = {0,0,0};

        // bool valid = false;
    };
    
    DeadReckoning() = default;

    void setParams(const Parameters &params){
        this->params = params;
        std::cout << "DeadReckoning: Params Loaded. " << std::endl << params.g_project.matrix() << std::endl;
    }

    void setIntegrationMode(IntegrationMode mode){state.integration_mode = mode;}

    void transformAcceleration(const Eigen::Vector3d &accel_imu,const Eigen::Quaternion<double> &q_iw, Eigen::Vector3d &accel_world);
    
    void process(const Eigen::Vector3d &accel_world, const ros::Time &stamp, double delta_t, const Eigen::Vector3d &dominant_velocity);
    nav_msgs::Odometry getOdometryMsg();
    sensor_msgs::Imu getAccelWorldMsg();
    // bool valid(){return state.valid;}

    void setState(const State &state){this->state = state;}
    State getState(){return state;}

    void reset();

private:
    State state;
    Parameters params;

    void integrate(const Eigen::Vector3d &accel_world, double delta_t);
};

void DeadReckoning::transformAcceleration(const Eigen::Vector3d &accel_imu,const Eigen::Quaternion<double> &q_iw, Eigen::Vector3d &accel_world){

    // calculate the gravity vector in the scaled & biased imu frame
    Eigen::Vector3d projected_g_imu_frame = params.g_project * q_iw.inverse() * params.g;
    
    // convert g into a up-pointing vector (ENU convention)
    const Eigen::Vector3d g = {0,0,projected_g_imu_frame.norm()};
    
    accel_world = q_iw * accel_imu - g;

    // accel = q * (accel - projected_g_imu_frame); // SUB-OPTIMAL
    // accel = q * accel_high_pass - g; //  High-pass version, SUB-OPTIMAL
    // accel = q * params.g_project.inverse() * accel - params.g; // SUB-OPTIMAL
}


void DeadReckoning::process(const Eigen::Vector3d &accel_world,const ros::Time &stamp, double delta_t, const Eigen::Vector3d &dominant_velocity){
    
    state.stamp = stamp;

    // Estimate Global Accelerometer Drift
    static Eigen::Vector3d low_pass = {0,0,0};
    const double alpha = 0.9;
    if (state.integration_mode == DOMINANT_DIRECTION){
        low_pass = alpha*low_pass + (1 - alpha) * dominant_velocity;
    }else
        low_pass = 0.99*low_pass;

    // std::cout << "low_pass = " << low_pass.transpose() << std::endl;

    state.accel = accel_world; // - accel_low_pass_bias;

    // Dominant Velocity and acceleration corrections
    if (state.integration_mode == STATIONARY)
    {
        // Trim velocity towards origin
        // state.velocity = (1.0/(1.0 + 10*delta_t))*state.velocity;

        if (state.velocity.norm() < 0.05)
            state.velocity = {0,0,0};
        else
            state.velocity = 1.0/(1.0 + 10*delta_t)*state.velocity; //- dominant_velocity*delta_t;
        // Set acceleration zero
        
        state.accel = {0,0,0};

    }else if (state.integration_mode == ACCEL_SMALL){
        
        // dominant velocity = average acceleration
        // if (dominant_velocity.norm()<0.12)
        //     state.accel = {0,0,0};
        
        if (state.velocity.norm() < 0.05)
            state.velocity = {0,0,0};
        else
            state.velocity = 1.0/(1.0 + delta_t)*state.velocity; //- low_pass*delta_t;

    }else if(state.integration_mode == DOMINANT_DIRECTION){
        // state.velocity -= low_pass*delta_t;
        // // // Trim velocity towards the dominant direction
        Eigen::Vector3d projected_velocity = dominant_velocity.normalized().dot(state.velocity)*dominant_velocity.normalized();
        double strength = 50.0/( 50.0 + dominant_velocity.norm());
        state.velocity = projected_velocity*(1-strength) + strength*state.velocity;

        // trim acceleration to only contain components in dominant_direction
        state.accel = dominant_velocity.normalized().dot(state.accel)*dominant_velocity.normalized();
        std::cout << "strength= " << strength << std::endl;
    }else if(state.integration_mode == GENERAL_MOTION){
        
        if (low_pass.norm() > 1e-3){
            Eigen::Vector3d projected_velocity = low_pass.normalized().dot(state.velocity)*low_pass.normalized();
            double strength = 50.0/( 50.0 + low_pass.norm());
            state.velocity = projected_velocity*(1-strength) + strength*state.velocity;
            std::cout << "strength= " << strength << std::endl;
        }
        
    }
    else{
        std::cout  << "Uncaught dead reckoning integration mode" << std::endl;
    }

    state.velocity = state.velocity +  state.accel * delta_t;
    state.position += state.velocity * delta_t +   0.5 * state.accel * delta_t * delta_t;
}

nav_msgs::Odometry DeadReckoning::getOdometryMsg(){
    nav_msgs::Odometry odometry;

    odometry.header.stamp = state.stamp;
    odometry.header.frame_id = "world_frame";

    odometry.pose.pose.position.x =  state.position(0);
    odometry.pose.pose.position.y =  state.position(1);
    odometry.pose.pose.position.z =  state.position(2);

    odometry.twist.twist.linear.x = state.velocity(0);
    odometry.twist.twist.linear.y = state.velocity(1); 
    odometry.twist.twist.linear.z = state.velocity(2); 

    return odometry;
}

sensor_msgs::Imu DeadReckoning::getAccelWorldMsg(){
    sensor_msgs::Imu imu;

    imu.header.stamp = state.stamp;
    imu.header.frame_id = "world_frame";

    imu.linear_acceleration.x = state.accel(0);
    imu.linear_acceleration.y = state.accel(1);
    imu.linear_acceleration.z = state.accel(2);

    imu.orientation.w = 1;
    imu.orientation.x = imu.orientation.y = imu.orientation.z = 0;

    return imu;
}

void DeadReckoning::reset(){
    state.position = {0,0,0};
    state.velocity = {0,0,0};
    state.accel = {0,0,0};
    // state.accel_bias = {0,0,0};
}

#endif /* DEAD_RECKONING_H */
