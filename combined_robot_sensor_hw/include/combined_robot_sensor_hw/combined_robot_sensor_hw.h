#ifndef COMBINED_ROBOT_SENSOR_HW_COMBINED_ROBOT_HW_H
#define COMBINED_ROBOT_SENSOR_HW_COMBINED_ROBOT_HW_H

#include <list>
#include <map>
#include <typeinfo>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/internal/interface_manager.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/sensor_hw.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <combined_robot_hw/combined_robot_hw.h>

namespace combined_robot_hw
{

/** \brief CombinedRobotSensorHW
 *
 * This class provides a way to combine RobotHW and Sensor objects.
 *
 *
 *
 */
class CombinedRobotSensorHW : public CombinedRobotHW
{
public:
  CombinedRobotSensorHW();

  virtual ~CombinedRobotSensorHW(){}
    /** \brief The init function is called to initialize the RobotHW from a
   * non-realtime thread.
   *
   * \param root_nh A NodeHandle in the root of the caller namespace.
   *
   * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
   * should read its configuration.
   *
   * \returns True if initialization was successful
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  /**
   * Reads data from the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref read
   */
  virtual void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Writes data to the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  //virtual void write(const ros::Time& time, const ros::Duration& period);


protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle robot_hw_nh_;
  pluginlib::ClassLoader<hardware_interface::SensorHW> sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::RobotHW> robot_hw_loader_;
  std::vector<boost::shared_ptr<hardware_interface::RobotHW> > robot_hw_list_;
  std::vector<boost::shared_ptr<hardware_interface::SensorHW> > sensor_list_;

  virtual bool loadSensorHW(const std::string& name);

};

}

#endif
