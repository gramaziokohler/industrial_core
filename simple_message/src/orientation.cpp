// TODO: Add license header

#ifndef FLATHEADERS
#include "simple_message/orientation.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "orientation.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace orientation
{

Orientation::Orientation(void)
{
  this->init();
}
Orientation::~Orientation(void)
{

}

void Orientation::init()
{
  this->init(0.0, 0.0, 0.0, 0.0);
}

void Orientation::init(industrial::shared_types::shared_real x, industrial::shared_types::shared_real y,
                       industrial::shared_types::shared_real z, industrial::shared_types::shared_real w)
{
  this->setX(x);
  this->setY(y);
  this->setZ(z);
  this->setW(w);
}

void Orientation::copyFrom(Orientation &src)
{
  this->setX(src.getX());
  this->setY(src.getY());
  this->setZ(src.getZ());
  this->setW(src.getW());
}

bool Orientation::operator==(Orientation &orientation)
{
  return this->x_ == orientation.x_ && this->y_ == orientation.y_ && this->z_ == orientation.z_ && this->w_ == orientation.w_;
}

bool Orientation::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing orientation load");

  if (buffer->load(this->x_) && buffer->load(this->y_) && buffer->load(this->z_) && buffer->load(this->w_))
  {
    LOG_COMM("Orientation successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("Orientation not loaded");
    rtn = false;
  }

  return rtn;
}

bool Orientation::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing position unload");
  if (buffer->unload(this->w_) && buffer->unload(this->z_) && buffer->unload(this->y_) && buffer->unload(this->x_))
  {
    rtn = true;
    LOG_COMM("Orientation successfully unloaded");
  }
  else
  {
    LOG_ERROR("Failed to unload orientation");
    rtn = false;
  }

  return rtn;
}

}
}

