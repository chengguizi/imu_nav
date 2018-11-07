#ifndef GRAVITY_ESTIMATOR_H
#define GRAVITY_ESTIMATOR_H
/**
 * @file gravity_estimator.h
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-20
 * 
 * @copyright Copyright (c) 2018
 * 
 */



#include <sensor_msgs/Imu.h>
#include <cassert>

#include <iostream>

#include "gravity_estimator_helper.h"


class GravityEstimator{
public:
    struct Parameters{
        int bucket_size = 16;
        int buffer_size = 128;
    };


    double xMax,xMin,yMax,yMin,zMax,zMin;
    sensor_msgs::Imu limits[6];

    GravityEstimator(int bucket_size, int buffer_size) : buffer_idx(0){
        params.bucket_size = bucket_size;
        params.buffer_size = buffer_size;
        buffer.resize(buffer_size, Bucket(bucket_size));

        xMax=xMin=yMax=yMin=zMax=zMin=std::numeric_limits<double>::quiet_NaN();
    }

    void push_back(const sensor_msgs::Imu &data){
        Bucket &current_bucket = buffer[buffer_idx];

        current_bucket.push_back(data);
        if (current_bucket.getIsFull()){ // the current bucket is full, ready for calculation.

            if (current_bucket.direction == NONE) // Current measurement is invalid, remove from the buffer
                current_bucket.reset();
            else{ // There is valid values, check consistency now
                if (buffer_idx == 0){
                    buffer_direction = current_bucket.direction;
                    std::cout << "\nNew direction: " << current_bucket.direction << std::endl;
                }else if(current_bucket.direction != buffer_direction){
                    reset();
                    return;
                }
                buffer_idx++;
                    std::cout << "\r" << "Bufferring: " << buffer_idx << "/" << params.buffer_size << std::flush;
            }
        }

        if (buffer_idx == params.buffer_size){
            // collect into vector
            std::vector<sensor_msgs::Imu> results;
            results.reserve(params.buffer_size);
            for ( auto bucket : buffer){
                results.push_back(bucket.avg);
            }

            sensor_msgs::Imu avg = calcMeanVariance(results);

            limits[(int)buffer_direction] = avg;
            displayLimits();

            reset();
        }
    }

    bool isDone(){
        return !( std::isnan(xMax) || std::isnan(xMin) || std::isnan(yMax) || std::isnan(yMin) || std::isnan(zMax) || std::isnan(zMin) );
    }

private:
    Parameters params;

    std::vector<Bucket> buffer;
    int buffer_idx;
    Direction buffer_direction;

    

    void reset(){
        buffer.clear();
        buffer.resize(params.buffer_size, Bucket(params.bucket_size));
        buffer_idx = 0;
        buffer_direction = NONE;
    }

    void displayLimits(){
        printf("\n\n=================================================================\n");
        printf("%10s %10s %10s %10s %10s %10s\n","[0]","[1]","[2]","[3]","[4]","[5]");
        printf("%10s %10s %10s %10s %10s %10s\n","X+","X-","Y+","Y-","Z+","Z-");

        
        if (limits[XPOS].linear_acceleration.x > 8)
            xMax = getAccelMagnitude(limits[XPOS]);
        if(limits[XNEG].linear_acceleration.x < -8)
            xMin = -getAccelMagnitude(limits[XNEG]);
        if(limits[YPOS].linear_acceleration.y > 8)
            yMax = getAccelMagnitude(limits[YPOS]);
        if(limits[YNEG].linear_acceleration.y < -8)
            yMin = -getAccelMagnitude(limits[YNEG]);
        if(limits[ZPOS].linear_acceleration.z > 8)
            zMax = getAccelMagnitude(limits[ZPOS]);
        if(limits[ZNEG].linear_acceleration.z < -8)
            zMin = -getAccelMagnitude(limits[ZNEG]);

        printf("%10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf\n",xMax,xMin,yMax,yMin,zMax,zMin);
        printf("=================================================================\n\n");
    }
};
#endif /* GRAVITY_ESTIMATOR_H */
