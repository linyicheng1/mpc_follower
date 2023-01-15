#include "demo/trajectory_publisher.h"
#include <ros/ros.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);

    while (ros::ok()){
        // tf sub

        // new trajectory

        // publish

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



