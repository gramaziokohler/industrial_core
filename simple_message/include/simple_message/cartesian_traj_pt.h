// TODO: Add license header

#ifndef CARTESIAN_TRAJ_PT_H
#define CARTESIAN_TRAJ_PT_H

#ifndef FLATHEADERS
#include "simple_message/orientation.h"
#include "simple_message/position.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "orientation.h"
#include "position.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

namespace industrial
{
namespace cartesian_traj_pt
{

namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  START_TRAJECTORY_DOWNLOAD  = -1, ///< Downloading drivers only: signal start of trajectory
  START_TRAJECTORY_STREAMING = -2, ///< Streaming drivers only: signal start of trajectory
  END_TRAJECTORY  = -3,            ///< Downloading drivers only: signal end of trajectory
  STOP_TRAJECTORY = -4             ///< Server should stop the current motion (if any) as soon as possible
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

/**
 * \brief Class encapsulated cartesian trajectory point data. The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * CartesianTrajectoryPoint message.
 *
 * This point differs from the ROS trajectory point in the following ways:
 *
 *  - The linear and angular velocities in an industrial robot standard way
 *    (as a single value).
 *  - The duration is somewhat different than the ROS timestamp.  The timestamp
 *    specifies when the move should start, where as the duration is how long the
 *    move should take.  A big assumption is that a sequence of points is continuously
 *    executed.  This is generally true of a ROS trajectory but not required.
 *
 * The byte representation of a cartesian trajectory point is as follow
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   position            (industrial::position)                    12 bytes
 *   orientation         (industrial::orientation)                 16 bytes
 *   linear_velocity     (industrial::shared_types::shared_real)   4  bytes
 *   angular_velocity    (industrial::shared_types::shared_real)   4  bytes
 *   acceleration        (industrial::shared_types::shared_real)   4  bytes
 *   blending_radius     (industrial::shared_types::shared_real)   4  bytes
 *   duration            (industrial::shared_types::shared_real)   4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class CartesianTrajPt : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  CartesianTrajPt(void);
  /**
   * \brief Destructor
   *
   */
  ~CartesianTrajPt(void);

  /**
   * \brief Initializes a empty cartesian trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory point
   *
   */
  void init(industrial::shared_types::shared_int sequence,
            industrial::position::Position & position,
            industrial::orientation::Orientation & orientation,
            industrial::shared_types::shared_real linear_velocity,
            industrial::shared_types::shared_real angular_velocity,
            industrial::shared_types::shared_real acceleration,
            industrial::shared_types::shared_real blending_radius,
            industrial::shared_types::shared_real duration);

  /**
   * \brief Sets cartesian trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns cartesian trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }

  /**
   * \brief Sets position
   *
   * \param position
   */
  void setPosition(industrial::position::Position &position)
  {
    this->position_.copyFrom(position);
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param position dest
   */
  void getPosition(industrial::position::Position &dest)
  {
    dest.copyFrom(this->position_);
  }

  /**
   * \brief Sets orientation
   *
   * \param orientation
   */
  void setOrientation(industrial::orientation::Orientation &orientation)
  {
    this->orientation_.copyFrom(orientation);
  }

  /**
   * \brief Returns a copy of the orientation data
   *
   * \param orientation dest
   */
  void getOrientation(industrial::orientation::Orientation &dest)
  {
    dest.copyFrom(this->orientation_);
  }

  /**
   * \brief Sets cartesian trajectory point linear velocity
   *
   * \param linear velocity
   */
  void setLinearVelocity(industrial::shared_types::shared_real linear_velocity)
  {
    this->linear_velocity_ = linear_velocity;
  }

  /**
   * \brief Returns cartesian trajectory point linear velocity
   *
   * \return cartesian trajectory point linear velocity
   */
  industrial::shared_types::shared_real getLinearVelocity()
  {
    return this->linear_velocity_;
  }

  /**
   * \brief Sets cartesian trajectory point angular velocity
   *
   * \param angular velocity
   */
  void setAngularVelocity(industrial::shared_types::shared_real angular_velocity)
  {
    this->angular_velocity_ = angular_velocity;
  }

  /**
   * \brief Returns cartesian trajectory point angular velocity
   *
   * \return cartesian trajectory point angular velocity
   */
  industrial::shared_types::shared_real getAngularVelocity()
  {
    return this->angular_velocity_;
  }

  /**
   * \brief Sets cartesian trajectory point acceleration
   *
   * \param acceleration
   */
  void setAcceleration(industrial::shared_types::shared_real acceleration)
  {
    this->acceleration_ = acceleration;
  }

  /**
   * \brief Returns cartesian trajectory point acceleration
   *
   * \return cartesian trajectory point acceleration
   */
  industrial::shared_types::shared_real getAcceleration()
  {
    return this->acceleration_;
  }

  /**
   * \brief Sets cartesian trajectory point blending radius
   *
   * \param blending radius
   */
  void setBlendingRadius(industrial::shared_types::shared_real blending_radius)
  {
    this->blending_radius_ = blending_radius;
  }

  /**
   * \brief Returns cartesian trajectory point blending radius
   *
   * \return cartesian trajectory point blending radius
   */
  industrial::shared_types::shared_real getBlendingRadius()
  {
    return this->blending_radius_;
  }

  /**
   * \brief Sets cartesian trajectory point duration
   *
   * \param velocity value
   */
  void setDuration(industrial::shared_types::shared_real duration)
  {
    this->duration_ = duration;
  }

  /**
   * \brief Returns cartesian trajectory point duration
   *
   * \return cartesian trajectory point duration
   */
  industrial::shared_types::shared_real getDuration()
  {
    return this->duration_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(CartesianTrajPt &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(CartesianTrajPt &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) +
           this->position_.byteLength() +
           this->orientation_.byteLength() +
           5 * sizeof(industrial::shared_types::shared_real);
  }

private:

  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

  /**
   * \brief cartesian point position
   */
  industrial::position::Position position_;

  /**
   * \brief cartesian point orientation
   */
  industrial::orientation::Orientation orientation_;

  /**
   * \brief linear velocity of the cartesian point
   */
  industrial::shared_types::shared_real linear_velocity_;

  /**
   * \brief angular velocity of the cartesian point
   */
  industrial::shared_types::shared_real angular_velocity_;

  /**
   * \brief acceleration of the cartesian point
   */
  industrial::shared_types::shared_real acceleration_;

  /**
   * \brief blending radius of the cartesian point
   */
  industrial::shared_types::shared_real blending_radius_;

  /**
   * \brief joint move duration
   */
  industrial::shared_types::shared_real duration_;
};

}
}

#endif /* CARTESIAN_TRAJ_PT_H */
