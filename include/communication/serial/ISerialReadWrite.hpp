#ifndef BUILDHAT_SRC_BUILDHAT_ISERIALREADWRITE_HPP_
#define BUILDHAT_SRC_BUILDHAT_ISERIALREADWRITE_HPP_

#include "globals.hpp"

#include "ISerialRead.hpp"
#include "ISerialWrite.hpp"

class ISerialReadWrite : virtual public ISerialRead, virtual public ISerialWrite {
 GEORDI_PUBLIC:
  ~ISerialReadWrite() override = default;
};

#endif //BUILDHAT_SRC_BUILDHAT_ISERIALREADWRITE_HPP_
