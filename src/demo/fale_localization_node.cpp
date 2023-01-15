#include "demo/fake_localization.h"
#include <list>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>

double pos_x, pos_y, pos_theta;
std::list<Eigen::Vector3d> trajectory;
FakeLocalization localization(1);

void publish_tf(float x, float y, float theta){
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0,0,theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster.sendTransform(transformStamped);
}

void publish_traj(ros::Publisher* publisher, std::list<Eigen::Vector3d>* data){
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    for (const auto& d:(*data)){
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "map";
        poseStamped.pose.position.x = d[0];
        poseStamped.pose.position.y = d[1];
        poseStamped.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0,0,d[2]);
        poseStamped.pose.orientation.x = q.x();
        poseStamped.pose.orientation.y = q.y();
        poseStamped.pose.orientation.z = q.z();
        poseStamped.pose.orientation.w = q.w();
        path.poses.emplace_back(poseStamped);
    }
    publisher->publish(path);
}

void diff_callback(const geometry_msgs::Twist::ConstPtr msg)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
    localization.DifferentialKinematicsUpdate(msg->linear.x, msg->angular.z, dt / 1000000.f);
    localization.GetPosition(pos_x, pos_y, pos_theta);
    Eigen::Vector3d pos;
    pos << pos_x, pos_y, pos_theta;

    static int delay = 0;
    if (delay > 100){
        trajectory.emplace_back(pos);
        delay = 0;
        if (trajectory.size() > 200){
            trajectory.pop_front();
        }
    }
    delay ++;

    last_time = now;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "fake_localization");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    ros::Publisher traj_pub = n.advertise<nav_msgs::Path>("trajectory", 1, true);
    ros::Subscriber diff_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, diff_callback);

    int delay = 0;
    while (ros::ok()){
        publish_tf(pos_x, pos_y, pos_theta);

        if (delay > 20){
            publish_traj(&traj_pub, &trajectory);
            delay = 0;
        }
        delay ++;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

