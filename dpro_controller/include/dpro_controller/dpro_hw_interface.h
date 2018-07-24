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
   Desc:   Example ros_control hardware interface that performs a perfect control loop for
   simulation
*/

#ifndef GENERIC_ROS_CONTROL__SIM_HW_INTERFACE_H
#define GENERIC_ROS_CONTROL__SIM_HW_INTERFACE_H

// C++
#include <string>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <dpro_controller/dynamixel_helper.h>

// Dynamixel
#include <dynamixel.h>
#include <bulkread.h>

namespace dpro_controller
{
/** \brief Hardware interface for a robot */
class DProHWInterface : public hardware_interface::RobotHW
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  DProHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);
  ~DProHWInterface();

  /** \brief Initialize the robot hardware interface */
  void init();

  bool read_servo_parameters(std::map<int, DynamixelInfo>& info_dynamixel);

  /** \brief Read the state from the robot hardware. */
  void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  void enforceLimits(ros::Duration& period);

  /** \brief Set all members to default values */
  void reset();

  /**
   * \brief Check (in non-realtime) if given controllers could be started and stopped from the
   * current state of the RobotHW
   * with regard to necessary hardware interface switches. Start and stop list are disjoint.
   * This is just a check, the actual switch is done in doSwitch()
   */
  bool canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                 const std::list<hardware_interface::ControllerInfo>& stop_list) const;

  /**
   * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
   * and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list);

  /**
   * \brief Register the limits of the joint specified by joint_id and joint_handle. The limits
   * are retrieved from the urdf_model.
   *
   * \return the joint's type, lower position limit, upper position limit, and effort limit.
   */
  void registerJointLimits(const hardware_interface::JointHandle& joint_handle_position,
                           const hardware_interface::JointHandle& joint_handle_velocity,
                           const hardware_interface::JointHandle& joint_handle_effort, std::size_t joint_id);

  /** \brief Helper for debugging a joint's state */
  void printState();

  std::string printStateHelper();

  /** \brief Helper for debugging a joint's command */
  std::string printCommandHelper();

  bool limitVelocityOfPositionControlledMotor(int id);
  bool sendPositionControlledMotorsToRestPosition();

protected:
  // Name of this class
  std::string name_;

  /** \brief Get the URDF XML from the parameter server */
  virtual void loadURDF(ros::NodeHandle& nh, std::string param_name);
  void loadTransmissions();
  void NEWloadTransmissions();
  void JointNEWloadTransmissions();

  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  hardware_interface::ActuatorStateInterface actuator_state_interface_;
  hardware_interface::PositionActuatorInterface actuator_position_interface_;
  hardware_interface::VelocityActuatorInterface actuator_velocity_interface_;
  hardware_interface::EffortActuatorInterface actuator_effort_interface_;

  // Joint limits interfaces - Saturation
  joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_interface_;
  joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_interface_;

  // Joint limits interfaces - Soft limits
  joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
  joint_limits_interface::EffortJointSoftLimitsInterface eff_jnt_soft_limits_;

  // Configuration
  std::vector<std::string> joint_names_;
  std::size_t num_joints_;
  urdf::Model* urdf_model_;
  std::string urdf_string_;
  // Modes
  bool use_rosparam_joint_limits_;
  bool use_soft_limits_if_available_;

  // Joint States
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  // Joint Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> previous_joint_position_command_;
  std::vector<double> previous_joint_velocity_command_;
  std::vector<double> previous_joint_effort_command_;

  // Copy of limits, in case we need them later in our control stack
  std::vector<double> joint_position_lower_limits_;
  std::vector<double> joint_position_upper_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_effort_limits_;

  // TODO: make this constant
  std::map<std::string, int> hardwareinterface2controlmode_;

  // Dynamixel Stuff
  std::string dynamixel_port_;
  int dynamixel_baudate_;

  DynamixelHelper* dynamixel_helper_;
  std::map<std::string, int> joint2id_;  // joint name <> dynamixel.id
  std::map<int, int> id2controlmode_;    // dynamixel.id <> control_mode
  std::map<int, DynamixelInfo> info_dynamixel_;
};  // class

}  // namespace

#endif
