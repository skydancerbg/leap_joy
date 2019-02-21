/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
//#include <std_msgs/Empty.h>

#include "std_msgs/String.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

using namespace std;

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_;
  double dead_zone_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  //ros::Publisher arduino_pub_;
  ros::Publisher mover_commands_pub_;
  ros::Publisher mover_vel_pub_;
  ros::Publisher emergency_pub_;
  ros::Subscriber joy_sub_;

  sensor_msgs::JointState joint_state_;
  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadarm_pressed_;
  bool deadman_pressed_;
  bool stop_pressed_;
  ros::Timer timer_;
    
  std_msgs::Bool stop_, start_;
};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(3),
  angular_(2),
  deadman_axis_(4),
  l_scale_(0.3),
  a_scale_(0.9),
  dead_zone_(0.1)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("joystick_dead_zone", dead_zone_, dead_zone_);

  //Solve bug documented in https://github.com/turtlebot/turtlebot_apps/issues/71
  deadman_pressed_ = false;
  deadarm_pressed_ = false;

  //arduino_pub_ = ph_.advertise<std_msgs::Empty>("/toggle_led", 1);
  mover_commands_pub_ = ph_.advertise<std_msgs::String>("CPRMoverCommands", 1);
  mover_vel_pub_ = ph_.advertise<sensor_msgs::JointState>("CPRMoverJointVel", 10);
  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);
  emergency_pub_ = ph_.advertise<std_msgs::Bool>("emergency_stop", 1);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
  
  stop_pressed_ = false;
  stop_.data = true;
  start_.data = false;
}
bool flag_ = true;
void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.angular.z = joy->axes[angular_];
  vel.linear.x = joy->axes[linear_];
  if (-dead_zone_ < vel.angular.z && vel.angular.z < dead_zone_)
    vel.angular.z = 0;
  if (-dead_zone_ < vel.linear.x && vel.linear.x < dead_zone_)
    vel.linear.x = 0;
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];

  bool stop = 0;
  //for (int i=0; i<4; ++i)
  //{
    stop |= joy->buttons[3];
  //}
  if (stop)
  {
    stop_pressed_ ^= true;
    if(stop_pressed_ == false)
    {
      emergency_pub_.publish(start_);
    }
  }
  /*
  if (joy->buttons[5]) {
    std_msgs::Empty empty;
    arduino_pub_.publish(empty);
  }
  */

  if (joy->buttons[0]) {
    std_msgs::String msg;
    msg.data = "Connect";
    mover_commands_pub_.publish(msg);
  }
  if (joy->buttons[1]) {
    std_msgs::String msg;
    msg.data = "Reset";
    mover_commands_pub_.publish(msg);
  }
  if (joy->buttons[2]) {
    std_msgs::String msg;
    msg.data = "Enable";
    mover_commands_pub_.publish(msg);
  }
  if (joy->buttons[6]) {
    std_msgs::String msg;
    msg.data = "GripperClose";
    mover_commands_pub_.publish(msg);
  }
  if (joy->buttons[7]) {
    std_msgs::String msg;
    msg.data = "GripperOpen";
    mover_commands_pub_.publish(msg);
  }
  

  if (joy->axes[0])
  {
  sensor_msgs::JointState joint_msg;
  vector<double> armvar(6, 0.0);
  armvar[0] = joy->axes[0] * 50;
  joint_msg.velocity = armvar;
  joint_state_ = joint_msg;
  //deadarm_pressed_ = joy->buttons[5];
  }
  if (joy->axes[1])
  {
  sensor_msgs::JointState joint_msg;
  vector<double> armvar(6, 0.0);
  armvar[1] = joy->axes[1] * 50;
  armvar[2] = armvar[1] * -1;
  joint_msg.velocity = armvar;
  joint_state_ = joint_msg;
  //deadarm_pressed_ = joy->buttons[5];
  }
  if (joy->axes[3])
  {
  sensor_msgs::JointState joint_msg;
  vector<double> armvar(6, 0.0);
  armvar[3] = joy->axes[3] * 50;
  armvar[2] = armvar[3] * -1;
  joint_msg.velocity = armvar;
  joint_state_ = joint_msg;
  //deadarm_pressed_ = joy->buttons[5];
  }
  deadarm_pressed_ = joy->buttons[5];
}

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    flag_ = true;
    vel_pub_.publish(last_published_);
  } 
  else
  {
    last_published_.angular.z = 0;
    last_published_.linear.x = 0;
    if (flag_)
    {
      vel_pub_.publish(last_published_);
      flag_ = false;
    }
  }
  if (stop_pressed_)
  {
    emergency_pub_.publish(stop_);
  }
  if (deadarm_pressed_)
  {
    //flag_ = true;
    mover_vel_pub_.publish(joint_state_);
  }
  else
  {
    vector<double> armvar(6, 0.0);
    //vector<double> joint_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    joint_state_.velocity = armvar;
    mover_vel_pub_.publish(joint_state_);
    
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  TurtlebotTeleop turtlebot_teleop;

  ros::spin();
}
