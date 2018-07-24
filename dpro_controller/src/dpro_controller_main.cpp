/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
Desc:   Example ros_control main() entry point for controlling robots in ROS
 */

//#include <dpro_controller/generic_hw_control_loop.h>
#include <dpro_controller/dpro_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dpro_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<dpro_controller::DProHWInterface> dpro_hw_interface(new dpro_controller::DProHWInterface(nh));
  dpro_hw_interface->init();

  controller_manager::ControllerManager cm(&(*dpro_hw_interface));
  ros::Rate loop_rate(50);

  ros::Time last_time = ros::Time::now();
  while (ros::ok())
  {
    // ROS_INFO("in main loop");
    loop_rate.sleep();

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    // ROS_INFO("before read");
    dpro_hw_interface->read(elapsed_time);
    // ROS_INFO("after read");

    // ROS_INFO("before cm.update");
    cm.update(current_time, elapsed_time);
    // ROS_INFO("after cm.update");

    // ROS_INFO("before write");
    dpro_hw_interface->write(elapsed_time);
    // ROS_INFO("after write");
  }
  //
  //  // Start the control loop
  //  dpro_controller::GenericHWControlLoop control_loop(nh, dpro_hw_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
