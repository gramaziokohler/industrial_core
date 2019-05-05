// TODO: Add license header

#ifndef FLATHEADERS
#include "simple_message/messages/cartesian_traj_pt_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "cartesian_traj_pt_message.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::cartesian_traj_pt;

namespace industrial
{
namespace cartesian_traj_pt_message
{

CartesianTrajPtMessage::CartesianTrajPtMessage(void)
{
  this->init();
}

CartesianTrajPtMessage::~CartesianTrajPtMessage(void)
{

}

bool CartesianTrajPtMessage::init(industrial::simple_message::SimpleMessage &msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (data.unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload cartesian traj pt data");
  }
  return rtn;
}

void CartesianTrajPtMessage::init(industrial::cartesian_traj_pt::CartesianTrajPt &point)
{
	this->init();
	this->point_.copyFrom(point);
}

void CartesianTrajPtMessage::init()
{
	this->setMessageType(StandardMsgTypes::CARTESIAN_TRAJ_PT);
  this->point_.init();
}

bool CartesianTrajPtMessage::load(ByteArray *buffer)
{
	LOG_COMM("Executing cartesian traj. pt. message load");

	if (!buffer->load(this->point_))
	{
		LOG_ERROR("Failed to load cartesian traj. pt data");
    return false;
	}

  return true;
}

bool CartesianTrajPtMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing cartesian traj pt message unload");

  if (!buffer->unload(this->point_))
  {
    LOG_ERROR("Failed to unload cartesian traj pt data");
    return false;
  }

  return true;
}

}
}

