#ifndef HARDWARE_INTERFACE_WSG50_GRASPING_INTERFACE_H
#define HARDWARE_INTERFACE_WSG50_GRASPING_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <cassert>
#include <boost/concept_check.hpp>

namespace hardware_interface
{
  
static const std::size_t num_args_ = 3;

/** \brief A handle used to read the state of a sdh tactile sensor.
 */
class WSG50GraspingHandle
{
public:
  WSG50GraspingHandle() : joint_name_(), target_position_(0),
    target_speed_(0), force_limit_(0), must_grasp_(0) {}

  WSG50GraspingHandle(std::string& joint_name, double *target_position,
    double *target_speed, double *force_limit, bool *must_grasp) : joint_name_(joint_name),
    target_position_(target_position), target_speed_(target_speed),
    force_limit_(force_limit), must_grasp_(must_grasp)
  {			 
    if (!target_position_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + joint_name + 
	"'. Target position data pointer is null.");
    }
    if (!target_speed_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + joint_name + 
	"'. Target speed data pointer is null.");
    }   
    if (!force_limit_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + joint_name + 
	"'. Force limit data pointer is null.");
    }
    if (!must_grasp_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + joint_name + 
	"'. Must grasp data pointer is null.");
    }
  }
  
  void grasp(std::vector<double> command)
  {
    assert(command.size() == num_args_);
    assert(target_position_); *target_position_ = command[0];
    assert(target_speed_); *target_speed_ = command[1];
    if (command[2] > 0)
    {
      assert(force_limit_); *force_limit_ = command[2];
    }
    assert(must_grasp_); *must_grasp_ = true;
  }
  std::string getName() const { return joint_name_; }
  std::size_t getNumberOfArgs() { return num_args_; }
  
private:
  std::string joint_name_;
  double *target_position_;
  double *target_speed_;
  double *force_limit_;
  bool *must_grasp_;
};

class WSG50GraspingInterface : public HardwareResourceManager<WSG50GraspingHandle> {};

}

#endif // HARDWARE_INTERFACE_WSG50_GRASPING_INTERFACE_H
