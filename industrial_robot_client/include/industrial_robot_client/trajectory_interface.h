﻿// TODO: Add license header

#ifndef TRAJECTORY_INTERFACE_H
#define TRAJECTORY_INTERFACE_H

#include <map>
#include <vector>
#include <string>
#include <boost/variant.hpp>

#include "ros/ros.h"
#include "industrial_msgs/CartesianTrajectory.h"
#include "industrial_msgs/CmdCartesianTrajectory.h"
#include "industrial_msgs/CmdJointTrajectory.h"
#include "industrial_msgs/StopMotion.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/messages/cartesian_traj_pt_message.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace industrial_robot_client
{
namespace trajectory_interface
{

using industrial::cartesian_traj_pt_message::CartesianTrajPtMessage;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * \brief Message handler that relays trajectories to the robot controller
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class TrajectoryInterface
{

public:

 /**
  * \brief Default constructor.
  */
    TrajectoryInterface() : default_joint_pos_(0.0), default_vel_ratio_(0.1), default_duration_(10.0) {};

    /**
     * \brief Initialize robot connection using default method.
     *
     * \param default_ip default IP address to use for robot connection [OPTIONAL]
     *                    - this value will be used if ROS param "robot_ip_address" cannot be read
     * \param default_port default port to use for robot connection [OPTIONAL]
     *                    - this value will be used if ROS param "~port" cannot be read
     *
     * \return true on success, false otherwise
     */
    virtual bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION);


    /**
     * \brief Initialize robot connection using specified method.
     *
     * \param connection new robot-connection instance (ALREADY INITIALIZED).
     *
     * \return true on success, false otherwise
     */
    virtual bool init(SmplMsgConnection* connection);

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
   * \return true on success, false otherwise (an invalid message type)
   */
    virtual bool init(SmplMsgConnection *connection, const std::vector<std::string> &joint_names,
                      const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

    virtual ~TrajectoryInterface();

    /**
   * \brief Begin processing messages and publishing topics.
   */
    virtual void run() { ros::spin(); }

  protected:
    /**
   * \brief Send a stop command to the robot
   */
    virtual void trajectoryStop();

    /**
   * \brief Convert ROS joint trajectory message into stream of JointTrajPtMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] traj ROS JointTrajectory message
   * \param[out] msgs list of JointTrajPtMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
    virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > *msgs);

    /**
   * \brief Convert ROS cartesian trajectory message into stream of CartesianTrajPtMessages for sending to robot.
   *
   * \param[in] traj ROS CartesianTrajectory message
   * \param[out] msgs list of CartesianTrajPtMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
    virtual bool trajectory_to_msgs(const industrial_msgs::CartesianTrajectoryConstPtr &traj, std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > *msgs);

    /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pt_in trajectory-point, in same order as expected for robot-connection.
   * \param[out] pt_out transformed trajectory-point (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
    virtual bool transform(const trajectory_msgs::JointTrajectoryPoint &pt_in, trajectory_msgs::JointTrajectoryPoint *pt_out)
    {
      *pt_out = pt_in; // by default, no transform is applied
      return true;
  }

  /**
   * \brief Transform cartesian positions before publishing.
   * Can be overridden to implement for robot-specific features.
   *
   * \param[in] pt_in trajectory-point, in same order as expected for robot-connection.
   * \param[out] pt_out transformed trajectory-point (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const industrial_msgs::CartesianTrajectoryPoint &pt_in, industrial_msgs::CartesianTrajectoryPoint *pt_out)
  {
    *pt_out = pt_in; // by default, no transform is applied
    return true;
  }

  /**
   * \brief Select cartesian data for sending to the robot
   *
   * \param[in] ros_pt target pos/vel from ROS command
   * \param[out] rbt_pt target pos/vel
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const industrial_msgs::CartesianTrajectoryPoint &ros_pt, industrial_msgs::CartesianTrajectoryPoint *rbt_pt);

  /**
   * \brief Select specific joints for sending to the robot
   *
   * \param[in] ros_joint_names joint names from ROS command
   * \param[in] ros_pt target pos/vel from ROS command
   * \param[in] rbt_joint_names joint names, in order/count expected by robot connection
   * \param[out] rbt_pt target pos/vel, matching rbt_joint_names
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<std::string>& ros_joint_names, const trajectory_msgs::JointTrajectoryPoint& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, trajectory_msgs::JointTrajectoryPoint* rbt_pt);

  /**
   * \brief Reduce the ROS velocity commands (per-joint velocities) to a single scalar for communication to the robot.
   *   For flexibility, the robot command message contains both "velocity" and "duration" fields.  The specific robot
   *   implementation can utilize either or both of these fields, as appropriate.
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_velocity computed velocity scalar for robot message (if needed by robot)
   * \param[out] rbt_duration computed move duration for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_speed(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity, double* rbt_duration);

  /**
   * \brief Compute velocity for communication to the robot.
   *   For flexibility, the robot command message contains both "velocity" and "duration" fields.  The specific robot
   *   implementation can utilize either or both of these fields, as appropriate.
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_velocity computed velocity scalar for robot message (if needed by robot)
   * \param[out] rbt_duration computed move duration for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_speed(const industrial_msgs::CartesianTrajectoryPoint &pt, double *rbt_velocity, double *rbt_duration);

  /**
   * \brief Reduce the ROS velocity commands (per-joint velocities) to a single scalar for communication to the robot.
   *   If unneeded by the robot server, set to 0 (or any value).
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_velocity computed velocity scalar for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_velocity(const trajectory_msgs::JointTrajectoryPoint &pt, double *rbt_velocity);

  /**
   * \brief Compute velocity for communication to the robot.
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_velocity computed velocity scalar for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_velocity(const industrial_msgs::CartesianTrajectoryPoint &pt, double *rbt_velocity);

  /**
   * \brief Compute the expected move duration for communication to the robot.
   *   If unneeded by the robot server, set to 0 (or any value).
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_duration computed move duration for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_duration(const boost::variant<trajectory_msgs::JointTrajectoryPoint, industrial_msgs::CartesianTrajectoryPoint> &pt, double *rbt_duration);

  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param messages List of SimpleMessage trajectory points to send to robot.
   *
   * \return true on success, false otherwise
   */
  virtual bool send_to_robot(const std::vector< boost::variant<CartesianTrajPtMessage, JointTrajPtMessage> > &messages) = 0;

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param msg JointTrajectory message from ROS trajectory-planner
   */
  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param msg CartesianTrajectory message from ROS trajectory-planner
   */
  virtual void cartesianTrajectoryCB(const industrial_msgs::CartesianTrajectoryConstPtr &msg);

  /**
   * \brief Callback function registered to ROS stopMotion service
   *   Sends stop-motion command to robot.
   *
   * \param req StopMotion request from service call
   * \param res StopMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool stopMotionCB(industrial_msgs::StopMotion::Request &req,
                            industrial_msgs::StopMotion::Response &res);

  /**
   * \brief Validate that trajectory command meets minimum requirements
   *
   * \param traj incoming trajectory
   * \return true if trajectory is valid, false otherwise
   */
  virtual bool is_valid(const trajectory_msgs::JointTrajectory &traj);

  /**
   * \brief Validate that trajectory command meets minimum requirements
   *
   * \param traj incoming trajectory
   * \return true if trajectory is valid, false otherwise
   */
  virtual bool is_valid(const industrial_msgs::CartesianTrajectory &traj);

  /*
   * \brief Callback for JointState topic
   *
   * \param msg JointState message
   */
  virtual void jointStateCB(const sensor_msgs::JointStateConstPtr &msg);

  TcpClient default_tcp_connection_;

  ros::NodeHandle node_;
  SmplMsgConnection* connection_;
  ros::Subscriber sub_cur_pos_;  // handle for joint-state topic subscription
  ros::Subscriber sub_joint_trajectory_;        // handle for joint-trajectory topic subscription
  ros::Subscriber sub_cartesian_trajectory_;    // handle for joint-trajectory topic subscription
  ros::ServiceServer srv_joint_trajectory_;     // handle for joint-trajectory service
  ros::ServiceServer srv_cartesian_trajectory_; // handle for cartesian-trajectory service
  ros::ServiceServer srv_stop_motion_;          // handle for stop_motion service
  std::vector<std::string> all_joint_names_;
  double default_joint_pos_;                    // default position to use for "dummy joints", if none specified
  double default_vel_ratio_;                    // default velocity ratio to use for joint commands, if no velocity or max_vel specified
  double default_duration_;                     // default duration to use for joint commands, if no
  std::map<std::string, double> joint_vel_limits_; // cache of max joint velocities from URDF
  sensor_msgs::JointState cur_joint_pos_;  // cache of last received joint state


private:
  static JointTrajPtMessage create_message(int seq, std::vector<double> joint_pos, double velocity, double duration);
  static CartesianTrajPtMessage create_message(int seq, double x, double y, double z, double rx, double ry, double rz, double rw,
                                               double velocity, double acceleration, double blending_radius, double duration);

  /**
   * \brief Callback function registered to ROS CmdJointTrajectory service
   *   Duplicates message-topic functionality, but in service form.
   *
   * \param req CmdJointTrajectory request from service call
   * \param res CmdJointTrajectory response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded
   */
      bool jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                             industrial_msgs::CmdJointTrajectory::Response &res);

  /**
   * \brief Callback function registered to ROS CmdCartesianTrajectory service
   *   Duplicates message-topic functionality, but in service form.
   *
   * \param req CmdCartesianTrajectory request from service call
   * \param res CmdCartesianTrajectory response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded
   */
      bool cartesianTrajectoryCB(industrial_msgs::CmdCartesianTrajectory::Request &req,
                                 industrial_msgs::CmdCartesianTrajectory::Response &res);
};

} //joint_trajectory_interface
} //industrial_robot_client

#endif /* TRAJECTORY_INTERFACE_H */
