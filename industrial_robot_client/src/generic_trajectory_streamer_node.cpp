// TODO: Add license header

#include "industrial_robot_client/trajectory_streamer.h"

using industrial_robot_client::trajectory_streamer::TrajectoryStreamer;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "motion_interface");

  // launch the default TrajectoryStreamer connection/handlers
  TrajectoryStreamer motionInterface;
  motionInterface.init();
  motionInterface.run();

  return 0;
}
