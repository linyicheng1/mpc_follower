#include "demo/trajectory_publisher.h"


TrajectoryPublisher::TrajectoryPublisher(int mode)
{
    mode_ = mode;
}

void TrajectoryPublisher::TrajectoryUpdate(double current_x, double current_y, double current_th)
{
    if (mode_  == 1){
        TrajectoryUpdateSin(current_x, current_y, current_th);
    }
    else if (mode_  == 1){
        TrajectoryUpdatePoly(current_x, current_y, current_th);
    }
}

void TrajectoryPublisher::TrajectoryUpdateSin(double current_x, double current_y, double current_th)
{

}

void TrajectoryPublisher::TrajectoryUpdatePoly(double current_x, double current_y, double current_th)
{

}
