//
// Created by qiayuan on 7/17/21.
//

#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <utility>

namespace rm_manual
{
class InputEvent
{
public:
  InputEvent() : last_state_(false)
  {
  }
  bool getState()
  {
    return last_state_;
  }
  //注册函数rising_handler()处理上升沿事件
  void setRising(boost::function<void()> handler)
  {
    //rising_handler_函数指针，存的是函数地址。move()是把handler（也是一个函数指针）地址右移给rising_handler_了
    rising_handler_ = std::move(handler);
  }
  void setFalling(boost::function<void()> handler)
  {
    falling_handler_ = std::move(handler);
  }
  void setActiveHigh(boost::function<void(ros::Duration)> handler)
  {
    active_high_handler_ = std::move(handler);
  }
  void setActiveLow(boost::function<void(ros::Duration)> handler)
  {
    active_low_handler_ = std::move(handler);
  }
  void setEdge(boost::function<void()> rising_handler, boost::function<void()> falling_handler)
  {
    rising_handler_ = std::move(rising_handler);
    falling_handler_ = std::move(falling_handler);
  }
  void setActive(boost::function<void(ros::Duration)> high_handler, boost::function<void(ros::Duration)> low_handler)
  {
    active_high_handler_ = std::move(high_handler);
    active_low_handler_ = std::move(low_handler);
  }
  void setDelayTriggered(boost::function<void()> delay_handler, double duration, bool is_rising_trigger)
  {
    ros::NodeHandle nh;
    delay_time_ = duration;
    triggered_timer_ = nh.createTimer(ros::Duration(delay_time_), std::bind(delay_handler), true, false);
    if (is_rising_trigger)
      trigger_rising_handler_ = boost::bind(&InputEvent::startTimer, this);
    else
      trigger_falling_handler_ = boost::bind(&InputEvent::startTimer, this);
  }
  void update(bool state)
  {
    //当前状态和上一次状态不同
    if (state != last_state_)
    {
      //如果当前状态是高电平且函数指针不为空，就调用rising_handler_()来处理上升沿事件
      if (state && rising_handler_)
        rising_handler_();
      else if (!state && falling_handler_)
        falling_handler_();
      if (state && trigger_rising_handler_)
        trigger_rising_handler_();
      else if (!state && trigger_falling_handler_)
        trigger_falling_handler_();
      last_state_ = state;
      last_change_ = ros::Time::now();
    }
    if (state && active_high_handler_)
      active_high_handler_(ros::Time::now() - last_change_);
    if (!state && active_low_handler_)
      active_low_handler_(ros::Time::now() - last_change_);
  }

private:
  void startTimer()
  {
    triggered_timer_.setPeriod(ros::Duration(delay_time_));
    triggered_timer_.start();
  }

  bool last_state_;
  double delay_time_;
  ros::Time last_change_;
  ros::Timer triggered_timer_;
  boost::function<void(ros::Duration)> active_high_handler_, active_low_handler_;
  boost::function<void()> rising_handler_, falling_handler_, trigger_rising_handler_, trigger_falling_handler_;
};

}  // namespace rm_manual
