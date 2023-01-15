
#ifndef MPC_FOLLOWER_FAKE_LOCALIZATION_H
#define MPC_FOLLOWER_FAKE_LOCALIZATION_H

#include <eigen3/Eigen/Core>


class FakeLocalization{
public:
    explicit FakeLocalization(double wheel_base);
    void DifferentialKinematicsUpdate(double cmd_v, double cmd_w, double dt);
    void BicycleKinematicsUpdate(double cmd_v, double cmd_th, double dt);
    void GetPosition(double& x, double& y, double& theta);
private:
    Eigen::Vector3d position_;
    double wheel_base_;
};



#endif //MPC_FOLLOWER_FAKE_LOCALIZATION_H
