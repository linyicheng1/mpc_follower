
#include "mpc_follower_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;
  ros::spin();
  return 0;
};
