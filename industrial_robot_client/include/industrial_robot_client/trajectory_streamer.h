// TODO: Add license header

#ifndef TRAJECTORY_STREAMER_H
#define TRAJECTORY_STREAMER_H

#include <boost/variant.hpp>
#include <boost/thread/thread.hpp>
#include "industrial_robot_client/trajectory_interface.h"

namespace industrial_robot_client
{
namespace trajectory_streamer
{

using industrial::cartesian_traj_pt_message::CartesianTrajPtMessage;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_robot_client::trajectory_interface::TrajectoryInterface;

namespace TransferStates
{
enum TransferState
{
  IDLE = 0, STREAMING =1 //,STARTING, //, STOPPING
};
}
typedef TransferStates::TransferState TransferState;

/**
 * \brief Message handler that streams joint & cartesian trajectories to the robot controller
 */

//* TrajectoryStreamer
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class TrajectoryStreamer : public TrajectoryInterface
{

public:

  // since this class defines a different init(), this helps find the base-class init()
  using TrajectoryInterface::init;

  /**
   * \brief Default constructor
   *
   * \param min_buffer_size minimum number of points as required by robot implementation
   */
  TrajectoryStreamer(int min_buffer_size = 1) : min_buffer_size_(min_buffer_size) {};

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \param linear_velocity_limit maximum linear velocity, used to compute tool linear speed during cartesian move
   * \param angular_velocity_limit maximum angular velocity, used to compute tool rotational speed during cartesian move
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>(),
                    double linear_velocity_limit = 0.0, double angular_velocity_limit = 0.0);

  ~TrajectoryStreamer();

  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
  virtual void cartesianTrajectoryCB(const industrial_msgs::CartesianTrajectoryConstPtr &msg);

  virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > *msgs);
  virtual bool trajectory_to_msgs(const industrial_msgs::CartesianTrajectoryConstPtr &traj, std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > *msgs);

  void streamingThread();

  bool send_to_robot(const std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > &messages);

protected:

  void trajectoryStop();

  boost::thread* streaming_thread_;
  boost::mutex mutex_;
  int current_point_;
  std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > current_traj_;
  TransferState state_;
  ros::Time streaming_start_;
  int min_buffer_size_;
};

} //trajectory_streamer
} //industrial_robot_client

#endif /* TRAJECTORY_STREAMER_H */
