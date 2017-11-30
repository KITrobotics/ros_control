#ifndef HARDWARE_INTERFACE_SDH_TEMPERATURE_INTERFACE_H
#define HARDWARE_INTERFACE_SDH_TEMPERATURE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <cassert>

namespace hardware_interface
{

/** \brief A handle used to read the state of a sdh tactile sensor.
 */
class SDHTemperatureHandle
{
public:
  SDHTemperatureHandle() : name_(""), temperature_value_(0) {}

  SDHTemperatureHandle(  const std::string& name, const double* temperature_value) : 
	name_(name), temperature_value_(temperature_value)
  {			   
    if (!temperature_value_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Temperature data pointer is null.");
    }
  }
  
  std::string getName()     const {return name_;}
  double getTemperature() const { assert(temperature_value_); return *temperature_value_; }
  
private:
  std::string name_;
  const double *temperature_value_;  
};

class SDHTemperatureInterface : public HardwareResourceManager<SDHTemperatureHandle> {};

}

#endif // HARDWARE_INTERFACE_SDH_TEMPERATURE_INTERFACE_H
