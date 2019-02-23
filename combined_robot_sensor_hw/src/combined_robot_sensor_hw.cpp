#include <algorithm>
#include <combined_robot_hw/combined_robot_hw.h>
#include <combined_robot_sensor_hw/combined_robot_sensor_hw.h>

namespace combined_robot_hw
{
  CombinedRobotSensorHW::CombinedRobotSensorHW() :
    sensor_loader_("hardware_interface","hardware_interface::SensorHW")
  {}


  bool CombinedRobotSensorHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
  {
    
    root_nh_ = root_nh;
    robot_hw_nh_ = robot_hw_nh;
    CombinedRobotHW::init(root_nh,robot_hw_nh);
   
    //sensor part     
    std::vector<std::string> sensors;
    std::string param_name = "sensor";
    if (!robot_hw_nh.getParam(param_name, sensors))
    {
      ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << robot_hw_nh.getNamespace() << ").");
      return false;
    }
    std::vector<std::string>::iterator it;
    for (it = sensors.begin(); it != sensors.end(); it++)
    {
      if (!loadSensorHW((*it)))
      {
        return false;
      }      
    }
    
    return true;
  }

  bool CombinedRobotSensorHW::loadSensorHW(const std::string& name)
  {
    
    ros::NodeHandle c_nh;
    // Constructs the sensor HW
    try
    {
      c_nh = ros::NodeHandle(robot_hw_nh_, name);
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for sensor HW with name '%s':\n%s", name.c_str(), e.what());
      return false;
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for sensor HW with name '%s'", name.c_str());
      return false;
    }         
      boost::shared_ptr<hardware_interface::SensorHW> sensor_hw;
      std::string type;
      if (c_nh.getParam("type", type))
      {        
        ROS_DEBUG("Constructing sensor HW '%s' of type '%s'", name.c_str(), type.c_str());
        try
        {
          std::vector<std::string> cur_types = sensor_loader_.getDeclaredClasses();
          for(size_t i=0; i < cur_types.size(); i++)
          {            
            if (type == cur_types[i])
            {
              sensor_hw = sensor_loader_.createInstance(type);
              
            }
          }
        }
        catch (const std::runtime_error &ex)
        {
          ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
        }
      }  
      else
      {
        ROS_ERROR("Could not load sensor hw '%s' because the type was not specified. Did you load the configuration on the parameter server (namespace: '%s')?", name.c_str(), c_nh.getNamespace().c_str());
        return false;
      }
      // checks if robot HW was constructed
      if (!sensor_hw)
      {
        ROS_ERROR("Could not load sensor hw '%s' because sensor hw type '%s' does not exist.",  name.c_str(), type.c_str());
        return false;
      }
      
      
      ROS_DEBUG("Initializing sensor '%s'", name.c_str());
      bool initialized;      
      try
      {         
        initialized = sensor_hw->init(root_nh_,c_nh);        
      }
      catch(std::exception &e)
      {
        ROS_ERROR("Exception thrown while initializing sensor %s.\n%s", name.c_str(), e.what());
        initialized = false;
      }
      catch(...)
      {
        ROS_ERROR("Exception thrown while initializing sensor %s", name.c_str());
        initialized = false;
      }

      if (!initialized)
      {
        ROS_ERROR("Initializing sensor '%s' failed", name.c_str());
        return false;
      }
      ROS_DEBUG("Initialized sensor'%s' successful", name.c_str());

      this->registerInterfaceManager(sensor_hw.get());
      sensor_list_.push_back(sensor_hw);      
      
      ROS_INFO("Successfully load sensor '%s'", name.c_str());
      return true;    
  }

  void CombinedRobotSensorHW::read(const ros::Time& time, const ros::Duration& period)
  {
    // Call the read method of the single RobotHW objects.
    std::vector<boost::shared_ptr<hardware_interface::RobotHW> >::iterator robot_hw;
    for (robot_hw = robot_hw_list_.begin(); robot_hw != robot_hw_list_.end(); ++robot_hw)
    {
      (*robot_hw)->read(time, period);
    }
    std::vector<boost::shared_ptr<hardware_interface::SensorHW>>::iterator sensor_hw;
    for (sensor_hw = sensor_list_.begin(); sensor_hw != sensor_list_.end(); ++sensor_hw)
    {
      (*sensor_hw)->read(time, period);
    }
  }

}
