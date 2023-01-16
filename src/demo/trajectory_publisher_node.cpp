#include "demo/trajectory_publisher.h"
#include <ros/ros.h>
#include "mpc_follower/MPCPath.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);
    mpc_follower::MPCPath path;
    while (ros::ok()){
        // tf sub

        // new trajectory

        // publish

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



