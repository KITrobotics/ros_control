#ifndef HARDWARE_INTERFACE_HANDLER_BASE_H
#define HARDWARE_INTERFACE_HANDLER_BASE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface
{

/// A handle base class
class HandlerBase
{
public:
  HandlerBase();

  virtual std::string getName();
};
    
}
#endif //HARDWARE_INTERFACE_HANDLER_BASE_H
