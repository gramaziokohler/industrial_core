
#ifndef ORIENTATION_H
#define ORIENTATION_H

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
namespace orientation
{

/**
 * \brief Class encapsulated cartesian orientation in quaternions.
 *
 * The byte representation of an orientation is as follows. The standard sizes
 * are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   x                   (industrial::shared_types::shared_real)   4
 *   y                   (industrial::shared_types::shared_real)   4
 *   z                   (industrial::shared_types::shared_real)   4
 *   w                   (industrial::shared_types::shared_real)   4
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class Orientation : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  Orientation(void);
  /**
   * \brief Destructor
   *
   */
  ~Orientation(void);

  /**
   * \brief Initializes a empty orientation
   *
   */
  void init();
  
  /**
   * \brief Initializes a full orientation
   *
   */
  void init(industrial::shared_types::shared_real x, industrial::shared_types::shared_real y,
            industrial::shared_types::shared_real z, industrial::shared_types::shared_real w);

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

  industrial::shared_types::shared_real getW()
  {
    return w_;
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

  void setW(industrial::shared_types::shared_real w)
  {
    this->w_ = w;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(Orientation &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(Orientation &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 4 * sizeof(industrial::shared_types::shared_real);
  }

private:
  industrial::shared_types::shared_real x_;
  industrial::shared_types::shared_real y_;
  industrial::shared_types::shared_real z_;
  industrial::shared_types::shared_real w_;
};
}
}

#endif /* ORIENTATION_H */
