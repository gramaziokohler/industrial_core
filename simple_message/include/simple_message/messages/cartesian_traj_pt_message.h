// TODO: Add license header

#ifndef CARTESIAN_TRAJ_PT_MESSAGE_H
#define CARTESIAN_TRAJ_PT_MESSAGE_H

#ifndef FLATHEADERS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
#include "simple_message/cartesian_traj_pt.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "cartesian_traj_pt.h"
#endif

namespace industrial
{
namespace cartesian_traj_pt_message
{


/**
 * \brief Class encapsulated cartesian trajectory point message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::cartesian_traj_pt::CartesianTrajPt data type.
 * The data portion of this typed message matches CartesianTrajPt.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class CartesianTrajPtMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  CartesianTrajPtMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~CartesianTrajPtMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a cartesian trajectory point structure
   *
   * \param cartesian trajectory point data structure
   *
   */
  void init(industrial::cartesian_traj_pt::CartesianTrajPt & point);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->point_.byteLength();
  }

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence) { point_.setSequence(sequence); }

  industrial::cartesian_traj_pt::CartesianTrajPt point_;

private:


};

}
}

#endif /* CARTESIAN_TRAJ_PT_MESSAGE_H */
