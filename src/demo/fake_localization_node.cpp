#include "demo/fake_localization.h"
#include <list>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"

double pos_x, pos_y, pos_theta;
double state_vx, state_angle;
std::list<Eigen::Vector3d> trajectory;
FakeLocalization localization(1);

void publish_tf(ros::Publisher* publisher, double x, double y, double theta){
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

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0;

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    publisher->publish(pose);
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

void diff_callback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
    localization.DifferentialKinematicsUpdate(msg->twist.linear.x, msg->twist.angular.z, dt / 1000000.f);
    state_vx = msg->twist.linear.x;
    state_angle = msg->twist.angular.z;

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

void bicycle_callback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();

    localization.BicycleKinematicsUpdate(msg->twist.linear.x, msg->twist.angular.z, dt / 1000000.f);
    state_vx = msg->twist.linear.x;
    state_angle = msg->twist.angular.z;
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
    ros::Subscriber diff_sub = n.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 10, diff_callback);
    ros::Subscriber bicycle_sub = n.subscribe<geometry_msgs::TwistStamped>("/twist_raw", 10, bicycle_callback);
    ros::Publisher state_pub = n.advertise<std_msgs::Float32MultiArray>("/vehicle_status", 1, true);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/current_pose", 1, true);

    int delay = 0;
    while (ros::ok()){
        publish_tf(&pose_pub, pos_x, pos_y, pos_theta);
        std_msgs::Float32MultiArray state;
        state.data.emplace_back(state_vx);
        state.data.emplace_back(state_angle);
        state_pub.publish(state);

        if (delay > 10){
            publish_traj(&traj_pub, &trajectory);
            delay = 0;
        }
        delay ++;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

