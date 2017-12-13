#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/sensor_hw.h>

namespace hardware_interface
{

class ForceTorqueSensorHW : public SensorHW
{
public:
  ForceTorqueSensorHW();
  virtual ~ForceTorqueSensorHW(){};
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  
  virtual bool stop() {return true;}
  virtual bool recover() {return true;}

protected:  
  ForceTorqueSensorInterface fts_interface_;
};

ForceTorqueSensorHW::ForceTorqueSensorHW()
{
}

bool ForceTorqueSensorHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{

  // Populate hardware interfaces
  fts_interface_.registerHandle(ForceTorqueSensorHandle());
  
  registerInterface(&fts_interface_);

  return true;
}


void ForceTorqueSensorHW::read(const ros::Time& time, const ros::Duration& period)
{
  ForceTorqueSensorHandle h1 = fts_interface_.getHandle("");
  h1.getForce();
  h1.getTorque();
}
 bool stop() {ROS_INFO("to be implemented"); return true;}
 bool recover() {ROS_INFO("to be implemented"); return true;};
}


