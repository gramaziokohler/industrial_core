// TODO: Add license header

#ifndef FLATHEADERS
#include "simple_message/position.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "position.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace position
{

Position::Position(void)
{
  this->init();
}
Position::~Position(void)
{

}

void Position::init()
{
  this->init(0.0, 0.0, 0.0);
}

void Position::init(industrial::shared_types::shared_real x, industrial::shared_types::shared_real y, industrial::shared_types::shared_real z)
{
  this->setX(x);
  this->setY(y);
  this->setZ(z);
}

void Position::copyFrom(Position &src)
{
  this->setX(src.getX());
  this->setY(src.getY());
  this->setZ(src.getZ());
}

bool Position::operator==(Position &pos)
{
  return this->x_ == pos.x_ && this->y_ == pos.y_ && this->z_ == pos.z_;
}

bool Position::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing position load");

  if (buffer->load(this->x_) && buffer->load(this->y_) && buffer->load(this->z_))
  {

    LOG_COMM("Position successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("Position not loaded");
    rtn = false;
  }

  return rtn;
}

bool Position::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing position unload");
  if (buffer->unload(this->z_) && buffer->unload(this->y_) && buffer->unload(this->x_))
  {

    rtn = true;
    LOG_COMM("Position successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload position");
    rtn = false;
  }

  return rtn;
}

}
}

