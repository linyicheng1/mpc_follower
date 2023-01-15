

#ifndef MPC_FOLLOWER_TRAJECTORY_PUBLISHER_H
#define MPC_FOLLOWER_TRAJECTORY_PUBLISHER_H


class TrajectoryPublisher{
public:
    explicit TrajectoryPublisher(int mode);
    void TrajectoryUpdate(double current_x, double current_y, double current_th);
private:
    void TrajectoryUpdateSin(double current_x, double current_y, double current_th);
    void TrajectoryUpdatePoly(double current_x, double current_y, double current_th);

    int mode_;
};

#endif //MPC_FOLLOWER_TRAJECTORY_PUBLISHER_H
