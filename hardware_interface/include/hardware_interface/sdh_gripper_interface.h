#ifndef HARDWARE_INTERFACE_SDH_GRIPPER_INTERFACE_H
#define HARDWARE_INTERFACE_SDH_GRIPPER_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <cassert>
#include <control_msgs/TactileSensor.h>

namespace hardware_interface
{

/** \brief A handle used to read the state of a sdh tactile sensor.
 */
class SDHGripperHandle
{
public:
  SDHGripperHandle() : name_(""), joint_names_(), targetAngles_(), 
		       tactile_sensor_matrices_(), currentAngles_(), joint_order_map_() {}

  SDHGripperHandle( const std::string& name, 
		    const std::vector<std::string>* joint_names,
		    std::vector<double>* targetAngles,
		    const std::vector<control_msgs::TactileMatrix>* 
		      tactile_sensor_matrices,
		    const std::vector<double>* currentAngles,
		    const std::map<std::string, int>* joint_order_map
		    ) : 
		    name_(name),
		    joint_names_(joint_names),
		    targetAngles_(targetAngles),
		    tactile_sensor_matrices_(tactile_sensor_matrices),
		    currentAngles_(currentAngles),
		    joint_order_map_(joint_order_map)
  {
    if (!joint_names_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Joint names data pointer is null.");
    }
    if (!targetAngles_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Target angles data pointer is null.");
    }
    if (!currentAngles_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Current angles data pointer is null.");
    }
    if (!tactile_sensor_matrices_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Tactile sensor matrices data pointer is null.");
    }
    if (!joint_order_map_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Joint order map data pointer is null.");
    }
  }
  
  std::string getName()     const {return name_;}
  std::size_t getNumberOfJoints() const { return joint_names_->size(); }
  std::size_t getNumberOfSensors() const { return tactile_sensor_matrices_->size(); }
  void setTargetAngles(std::vector<double> targetAngles) 
  { 
    for (int i = 0; i < targetAngles_->size(); ++i)
    {
      int index = joint_order_map_->find(joint_names_->at(i))->second;
      targetAngles_->at(i) = targetAngles[index];
    }
  }
  std::vector<control_msgs::TactileMatrix> getMatrices() const 
    { assert(tactile_sensor_matrices_); return *tactile_sensor_matrices_; }
  std::vector<double> getCurrentAngles() const { 
    assert(currentAngles_); 
    std::vector<double> toReturn;
    toReturn.resize(joint_names_->size());
    for (int i = 0; i < joint_names_->size(); ++i)
    {
      int index = joint_order_map_->find(joint_names_->at(i))->second;
      toReturn[index] = currentAngles_->at(i);
    }
    return toReturn; 
  }
  std::map<std::string, int> getJointOrderMap() const 
    { assert(joint_order_map_); return *joint_order_map_; }
  
private:
  std::string name_;
  const std::vector<std::string>* joint_names_;
  std::vector<double>* targetAngles_;  
  const std::vector<double>* currentAngles_; 
  const std::vector<control_msgs::TactileMatrix>* tactile_sensor_matrices_;
  const std::map<std::string, int>* joint_order_map_;
};

class SDHGripperInterface : public HardwareResourceManager<SDHGripperHandle> {};

}

#endif // HARDWARE_INTERFACE_SDH_GRIPPER_INTERFACE_H