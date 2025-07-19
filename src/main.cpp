//
// Created by luohx on 7/27/20.
//

#include "rm_manual/manual_base.h"
#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"
#include "rm_manual/engineer_manual.h"
#include "rm_manual/engineer2_manual.h"
#include "rm_manual/dart_manual.h"
#include "rm_manual/wheeled_balance_manual.h"
#include "rm_manual/legged_wheel_balance_manual.h"

int main(int argc, char** argv)
{
  std::string robot;
  //manual_control是一个基类指针，可以指向任何rm_manual::ManualBase的子类对象
  rm_manual::ManualBase* manual_control;
  ros::init(argc, argv, "rm_manual");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_referee("rm_referee");
  robot = getParam(nh, "robot_type", (std::string) "error");
  //根据不同的robot_type选择不同的操作逻辑
  //通过new创建子类对象，并将其地址赋值给manual_control（基类指针）。通过基类指针调用方法时，C++会根据对象的实际类型（而非指针类型）来决定调用哪个方法
  if (robot == "standard")
    manual_control = new rm_manual::ChassisGimbalShooterCoverManual(nh, nh_referee);
  else if ((robot == "hero") || (robot == "drone"))
    manual_control = new rm_manual::ChassisGimbalShooterManual(nh, nh_referee);
  else if (robot == "engineer")
    manual_control = new rm_manual::EngineerManual(nh, nh_referee);
  else if (robot == "engineer2")
    manual_control = new rm_manual::Engineer2Manual(nh, nh_referee);
  else if (robot == "dart")
    manual_control = new rm_manual::DartManual(nh, nh_referee);
  else if (robot == "wheeled_balance")
    manual_control = new rm_manual::WheeledBalanceManual(nh, nh_referee);
  else if (robot == "legged_wheel_balance")
    manual_control = new rm_manual::LeggedWheelBalanceManual(nh, nh_referee);
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    manual_control->run();
    loop_rate.sleep();
  }
  return 0;
}
