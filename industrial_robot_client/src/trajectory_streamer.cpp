// TODO: Add license header

#include "industrial_robot_client/trajectory_streamer.h"

namespace industrial_robot_client
{
namespace trajectory_streamer
{

using industrial::simple_message::SimpleMessage;

class RequestVisitor : public boost::static_visitor<SimpleMessage>
{
public:
  template <typename T>
  SimpleMessage operator()(T &pt) const
  {
    SimpleMessage msg;
    pt.toRequest(msg);
    return msg;
  }
};

bool TrajectoryStreamer::init(SmplMsgConnection *connection, const std::vector<std::string> &joint_names,
                              const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("TrajectoryStreamer: init");

  rtn &= TrajectoryInterface::init(connection, joint_names, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
      new boost::thread(boost::bind(&TrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

TrajectoryStreamer::~TrajectoryStreamer()
{
  delete this->streaming_thread_;
}

void TrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

	this->mutex_.lock();
    trajectoryStop();
	this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

void TrajectoryStreamer::cartesianTrajectoryCB(const industrial_msgs::CartesianTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving cartesian trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

	this->mutex_.lock();
    trajectoryStop();
	this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

bool TrajectoryStreamer::send_to_robot(const std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > &messages)
{
  ROS_INFO("Loading trajectory, setting state to streaming");
  this->mutex_.lock();
  {
    ROS_INFO("Executing trajectory of size: %d", (int)messages.size());
    this->current_traj_.clear();
    for (size_t i = 0; i < messages.size(); ++i)
    {
      this->current_traj_.push_back(messages[i]);
    }
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->streaming_start_ = ros::Time::now();
  }
  this->mutex_.unlock();

  return true;
}

bool TrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > *msgs)
{
  // use base function to transform points
  if (!TrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}

bool TrajectoryStreamer::trajectory_to_msgs(const industrial_msgs::CartesianTrajectoryConstPtr &traj, std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > *msgs)
{
  // use base function to transform points
  if (!TrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}

void TrajectoryStreamer::streamingThread()
{
  boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> streamMsg;
  int connectRetryCount = 1;

  ROS_INFO("Starting trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, reply;
        
    switch (this->state_)
    {
      case TransferStates::IDLE:
        ros::Duration(0.010).sleep();  //  loop while waiting for new trajectory
        break;

      case TransferStates::STREAMING:
        if (this->current_point_ >= (int)this->current_traj_.size())
        {
          ROS_INFO("Trajectory streaming complete, setting state to IDLE");
          this->state_ = TransferStates::IDLE;
          break;
        }

        if (!this->connection_->isConnected())
        {
          ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        streamMsg = this->current_traj_[this->current_point_];
        msg = boost::apply_visitor(RequestVisitor(), streamMsg);

        ROS_DEBUG("Sending trajectory point");
        if (this->connection_->sendAndReceiveMsg(msg, reply, true))
        {
          this->current_point_++;
          ROS_INFO("Point[%d of %d] sent to controller",
                   this->current_point_, (int)this->current_traj_.size());
        }
        else
          ROS_WARN("Failed sent trajectory point, will try again");

        break;
      default:
        ROS_ERROR("Trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }

    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

void TrajectoryStreamer::trajectoryStop()
{
  TrajectoryInterface::trajectoryStop();

  ROS_DEBUG("Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;
}

} //trajectory_streamer
} //industrial_robot_client

