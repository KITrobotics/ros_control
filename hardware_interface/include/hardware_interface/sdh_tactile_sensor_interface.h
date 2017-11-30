#ifndef HARDWARE_INTERFACE_SDH_TACTILE_SENSOR_INTERFACE_H
#define HARDWARE_INTERFACE_SDH_TACTILE_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <cassert>
#include <control_msgs/TactileSensor.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hardware_interface
{

/** \brief A handle used to read the state of a sdh tactile sensor.
 */
class SDHTactileSensorHandle
{
public:
  SDHTactileSensorHandle() : name_(""), matrix_(), marker_() {}

  SDHTactileSensorHandle(const std::string& name, 
			 const control_msgs::TactileMatrix* matrix,
			 const visualization_msgs::Marker* marker) : 
			 matrix_(matrix),
			 name_(name),
			 marker_(marker)
  {
    if (!matrix_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Tactile matrix data pointer is null.");
    }
    if (!marker_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Marker array data pointer is null.");
    }
  }
  
  std::string getName()     const {return name_;}
  control_msgs::TactileMatrix getMatrix() const { assert(matrix_); return *matrix_; }
  visualization_msgs::Marker getMarker() const { assert(marker_); return *marker_; }
  
private:
  std::string name_;
  const control_msgs::TactileMatrix* matrix_;  
  const visualization_msgs::Marker* marker_;
};

class SDHTactileSensorInterface : public HardwareResourceManager<SDHTactileSensorHandle> {};

}

#endif // HARDWARE_INTERFACE_SDH_TACTILE_SENSOR_INTERFACE_H
