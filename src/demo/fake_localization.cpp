#include "demo/fake_localization.h"

FakeLocalization::FakeLocalization(double wheel_base)
{
    // init position
    position_ << 0, 0, 0;
    wheel_base_ = wheel_base;
}

/**
 * @brief update position by cmd using differential kinematics model
 * @param cmd_v
 * @param cmd_w
 * @param dt
 */
void FakeLocalization::DifferentialKinematicsUpdate(double cmd_v, double cmd_w, double dt)
{
    double yaw = position_[2] + 0.5 * cmd_w * dt;
    position_[0] = (position_[0] + cmd_v * cos(yaw) * dt);
    position_[1] = (position_[1] + cmd_v * sin(yaw) * dt);
    position_[2] += cmd_w * dt;
}

void FakeLocalization::BicycleKinematicsUpdate(double cmd_v, double cmd_th, double dt)
{
    double delta_yaw = cmd_v / wheel_base_ * std::tan(cmd_th) * dt;

    double yaw = position_[2] + 0.5 * delta_yaw;
    position_[0] = (position_[0] + cmd_v * cos(yaw) * dt);
    position_[1] = (position_[1] + cmd_v * sin(yaw) * dt);
    position_[2] += delta_yaw;
}

void FakeLocalization::GetPosition(double &x, double &y, double &theta)
{
    x = position_[0];
    y = position_[1];
    theta = position_[2];
}




