/*
Template code for subscribe and publish node

*/


#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <vector>
#include <string>

#include "math.h"

enum turn_direction {right, left};

#define DISPARITY 0.1 // meters
#define CAR_WIDTH 0.4
#define PI 3.1415926535



class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);        

        //Topic you want to subscribe
        sub_ = n_.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {

        turn_direction turn_dir;
        angle_increment = lidar_info.angle_increment;

        ranges = std::vector<double>(std::begin(lidar_info.ranges), std::end(lidar_info.ranges));
        double min_angle = -100 / 180.0 * PI;
        size_t min_indx = (size_t)(std::floor((min_angle - lidar_info.angle_min) / lidar_info.angle_increment));
        double max_angle = 100 / 180.0 * PI;
        size_t max_indx = (size_t)(std::ceil((max_angle - lidar_info.angle_min) / lidar_info.angle_increment));

        // Clean data
        for (size_t i = min_indx; i < max_indx; i++) {
            if (lidar_info.ranges[i] > lidar_info.range_max || std::isnan(lidar_info.ranges[i])) {
                ranges[i] = 0.0;
            } else if (std::isinf(lidar_info.ranges[i])) {
                ranges[i] = lidar_info.range_max;
            }
        }

        double distance;
        double last_distance;
        size_t interval;
        last_distance = ranges[min_indx];
        for (size_t i = min_indx+1; i < max_indx; i++) {
            if (ranges[i] == 0) continue;

            if (ranges[i] >= last_distance + DISPARITY) {
                distance = ranges[i];
                interval = i + getLidarIncrements(ranges[i]);
                for (; i < interval; i++)
                    ranges[i] = distance;
            }

            else if (ranges[i] <= last_distance - DISPARITY){
                distance = ranges[i];
                i = i - getLidarIncrements(ranges[i]);
                interval = i + getLidarIncrements(ranges[i]);
                for (; i < interval; i++)
                    ranges[i] = distance;
            }

            last_distance = ranges[i];
            
        }

        double max_value = ranges[min_indx];
        angle = 0;
        for (unsigned int i = min_indx + 1; i < max_indx; i++) {
            if (ranges[i] > max_value) { 
                max_value = ranges[i];
                angle = lidar_info.angle_min + i * lidar_info.angle_increment;
            }
            
        }

        speed = max_speed;
        if (max_value < 2) speed = max_speed / 2;
        
        SubscribeAndPublish::reactive_control();
    }

    void reactive_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = angle;
        ackermann_drive_result.drive.speed = speed;
        pub_.publish(ackermann_drive_result);
    }

    size_t getLidarIncrements(double distance){
        double theta = CAR_WIDTH / distance;
        return (size_t) ceil(theta / angle_increment);

        // s= r0
        // width + tolerance = r0 - our width includes tolerance
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double angle, speed;
    double angle_increment;
    double max_speed = 2;
    double max_steering_angle;
    std::vector<double> ranges;

};

int main(int argc, char** argv) {
    //Initiate ROS
    ros::init(argc, argv, "random_walker");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}