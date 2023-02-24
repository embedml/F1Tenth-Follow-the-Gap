/*
Template code for subscribe and publish node

Can add more subscribers/publishers - just change what node
*/


#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <vector>

#include "math.h"


class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);        

        //Topic you want to subscribe
        sub_ = n_.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {

        SubscribeAndPublish::reactive_control();
    }

    void reactive_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = angle;
        ackermann_drive_result.drive.speed = speed;
        pub_.publish(ackermann_drive_result);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double angle;
    double speed;
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