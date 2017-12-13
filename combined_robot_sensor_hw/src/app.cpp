#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <combined_robot_sensor_hw/combined_robot_sensor_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DummyApp");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");
  std::cout<<"nh: "<<nh.getNamespace()<<std::endl;
  combined_robot_hw::CombinedRobotSensorHW hw;
  bool init_success = hw.init(nh, nh);

  controller_manager::ControllerManager cm(&hw, nh);

  ros::Duration period(1.0);
  while (ros::ok())
  {
    hw.read(ros::Time::now(), period);
    cm.update(ros::Time::now(), period);
    hw.write(ros::Time::now(), period);
    period.sleep();
  }
}
