// TODO: Add license header

#ifndef FLATHEADERS
#include "simple_message/cartesian_traj_pt.h"
#include "simple_message/orientation.h"
#include "simple_message/position.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "cartesian_traj_pt.h"
#include "orientation.h"
#include "position.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::position;
using namespace industrial::orientation;
using namespace industrial::shared_types;

namespace industrial
{
namespace cartesian_traj_pt
{

CartesianTrajPt::CartesianTrajPt(void)
{
  this->init();
}
CartesianTrajPt::~CartesianTrajPt(void)
{

}

void CartesianTrajPt::init()
{
  this->position_.init();
  this->orientation_.init();
  this->sequence_ = 0;
  this->linear_velocity_ = 0.0;
  this->angular_velocity_ = 0.0;
  this->blending_radius_ = 0.0;
  this->duration_ = 0.0;
}

void CartesianTrajPt::init(shared_int sequence,
                           Position &position,
                           Orientation &orientation,
                           shared_real linear_velocity,
                           shared_real angular_velocity,
                           shared_real blending_radius,
                           shared_real duration)
{
  this->setSequence(sequence);
  this->setPosition(position);
  this->setOrientation(orientation);
  this->setLinearVelocity(linear_velocity);
  this->setAngularVelocity(angular_velocity);
  this->setBlendingRadius(blending_radius);
  this->setDuration(duration);
}

void CartesianTrajPt::copyFrom(CartesianTrajPt &src)
{
  this->setSequence(src.getSequence());
  src.getPosition(this->position_);
  src.getOrientation(this->orientation_);
  this->setLinearVelocity(src.getLinearVelocity());
  this->setAngularVelocity(src.getAngularVelocity());
  this->setBlendingRadius(src.getBlendingRadius());
  this->setDuration(src.getDuration());
}

bool CartesianTrajPt::operator==(CartesianTrajPt &other)
{
  return this->sequence_ == other.sequence_ &&
         this->position_ == other.position_ &&
         this->orientation_ == other.orientation_ &&
         this->linear_velocity_ == other.linear_velocity_ &&
         this->angular_velocity_ == other.angular_velocity_ &&
         this->blending_radius_ == other.blending_radius_ &&
         this->duration_ == other.duration_;
}

bool CartesianTrajPt::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing cartesian trajectory point load");

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load joint traj. pt. sequence number");
    return false;
  }

  if (!this->position_.load(buffer))
  {
    LOG_ERROR("Failed to load cartesian traj. pt. position data");
    return false;
  }

  if (!this->orientation_.load(buffer))
  {
    LOG_ERROR("Failed to load cartesian traj. pt. orientation data");
    return false;
  }

  if (!buffer->load(this->linear_velocity_))
  {
    LOG_ERROR("Failed to load cartesian traj. pt. linear velocity data");
    return false;
  }

  if (!buffer->load(this->angular_velocity_))
  {
    LOG_ERROR("Failed to load cartesian traj. pt. angular velocity data");
    return false;
  }

  if (!buffer->load(this->blending_radius_))
  {
    LOG_ERROR("Failed to load cartesian traj. pt. blending radious data");
    return false;
  }

  if (!buffer->load(this->duration_))
  {
    LOG_ERROR("Failed to load cartesian traj. pt. duration");
    return false;
  }

  return true;
}

bool CartesianTrajPt::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing cartesian traj. pt. unload");

  if (!buffer->unload(this->duration_))
  {
    LOG_ERROR("Failed to unload cartesian traj. pt. duration");
    return false;
  }

  if (!buffer->unload(this->blending_radius_))
  {
    LOG_ERROR("Failed to unload cartesian traj. pt. blending radious data");
    return false;
  }

  if (!buffer->unload(this->angular_velocity_))
  {
    LOG_ERROR("Failed to unload cartesian traj. pt. angular velocity data");
    return false;
  }

  if (!buffer->unload(this->linear_velocity_))
  {
    LOG_ERROR("Failed to unload cartesian traj. pt. linear velocity data");
    return false;
  }

  if (!this->orientation_.unload(buffer))
  {
    LOG_ERROR("Failed to unload cartesian traj. pt. orientation data");
    return false;
  }

  if (!this->position_.unload(buffer))
  {
    LOG_ERROR("Failed to unload cartesian traj. pt. position data");
    return false;
  }

  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload joint traj. pt. sequence number");
    return false;
  }

  return true;
}

}
}
