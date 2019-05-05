
#ifndef POSITION_H
#define POSITION_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

namespace industrial
{
namespace position
{

/**
 * \brief Class encapsulated cartesian position.
 *
 * The byte representation of a position is as follows. The standard sizes
 * are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   x                   (industrial::shared_types::shared_real)   4
 *   y                   (industrial::shared_types::shared_real)   4
 *   z                   (industrial::shared_types::shared_real)   4
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class Position : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  Position(void);
  /**
   * \brief Destructor
   *
   */
  ~Position(void);

  /**
   * \brief Initializes a empty position
   *
   */
  void init();
  
  /**
   * \brief Initializes a full position
   *
   */
  void init(industrial::shared_types::shared_real x, industrial::shared_types::shared_real y, industrial::shared_types::shared_real z);

  industrial::shared_types::shared_real getX()
  {
    return x_;
  }

  industrial::shared_types::shared_real getY()
  {
    return y_;
  }

  industrial::shared_types::shared_real getZ()
  {
    return z_;
  }

  void setX(industrial::shared_types::shared_real x)
  {
    this->x_ = x;
  }

  void setY(industrial::shared_types::shared_real y)
  {
    this->y_ = y;
  }

  void setZ(industrial::shared_types::shared_real z)
  {
    this->z_ = z;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(Position &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(Position &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 3 * sizeof(industrial::shared_types::shared_real);
  }

private:
  industrial::shared_types::shared_real x_;
  industrial::shared_types::shared_real y_;
  industrial::shared_types::shared_real z_;
};

}
}

#endif /* POSITION_H */
