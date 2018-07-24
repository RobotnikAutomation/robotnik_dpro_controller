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

#include <dpro_controller/dpro_hw_interface.h>

namespace dpro_controller
{
DProHWInterface::DProHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : name_("dpro_hw_interface")
  , nh_(nh)
  , pnh_("~")
  , use_rosparam_joint_limits_(false)
  , use_soft_limits_if_available_(true)
{
  pnh_.param<std::string>("port", dynamixel_port_, "/dev/ttyUSB0");
  pnh_.param<int>("baudrate", dynamixel_baudate_, 57600);

  // Check if the URDF model needs to be loaded
  if (urdf_model == NULL)
    loadURDF(nh, "robot_description");
  else
    urdf_model_ = urdf_model;
}

DProHWInterface::~DProHWInterface()
{
  sendPositionControlledMotorsToRestPosition();
  ros::Duration(5).sleep();
  dynamixel_helper_->disconnect();
  delete dynamixel_helper_;
}

bool DProHWInterface::read_servo_parameters(std::map<int, DynamixelInfo>& info_dynamixel)
{
  info_dynamixel.clear();

  // read in the information regarding the servos that we're supposed to connect to
  if (pnh_.hasParam("servos"))
  {
    XmlRpc::XmlRpcValue servos;
    pnh_.getParam("servos", servos);
    // If there is no servos array in the param server, return
    if (!(servos.getType() == XmlRpc::XmlRpcValue::TypeArray))
    {
      ROS_ERROR("Invalid/missing servo information on the param server");
      ROS_BREAK();
    }

    // For every servo, load and verify its information
    for (int i = 0; i < servos.size(); i++)
    {
      DynamixelInfo info;
      if (!(servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct))
      {
        ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
        ROS_BREAK();
      }

      if (!(servos[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR("Invalid/Missing id for servo index %d", i);
        ROS_BREAK();
      }
      else
      {
        // store the servo's ID
        info.id = static_cast<int>(servos[i]["id"]);
        ROS_INFO("id = %d", info.id);
      }

      if (!(servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString))
      {
        ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        // store the servo's corresponding joint
        info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);
      }

      if (!(servos[i]["model_name"].getType() == XmlRpc::XmlRpcValue::TypeString))
      {
        ROS_ERROR("Invalid/Missing model_name for servo index %d, id: %d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.model_name = static_cast<std::string>(servos[i]["model_name"]);
      }

      // uint16_t model_number;
      // uint32_t model_info;
      if (!(servos[i]["cpr"].getType() == XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR("Invalid/Missing cpr for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.cpr = static_cast<int>(servos[i]["cpr"]);
      }

      if (!(servos[i]["gear_reduction"].getType() == XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR("Invalid/Missing gear_reduction for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.gear_reduction = static_cast<int>(servos[i]["gear_reduction"]);
      }

      if (!(servos[i]["max_vel_value"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_ERROR("Invalid/Missing max_vel_value for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.max_vel_value = static_cast<double>(servos[i]["max_vel_value"]);
      }

      if (!(servos[i]["max_vel_rads"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_ERROR("Invalid/Missing max_vel_rads for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.max_vel_rads = static_cast<double>(servos[i]["max_vel_rads"]);
      }

      if (!(servos[i]["joint_type"].getType() == XmlRpc::XmlRpcValue::TypeString))
      {
        ROS_ERROR("Invalid/Missing type for servo index %d, id: %d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.joint_type = static_cast<std::string>(servos[i]["joint_type"]);
      }

      if (!(servos[i]["min_rad"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_ERROR("Invalid/Missing min_rad for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.min_rad = static_cast<double>(servos[i]["min_rad"]);
      }

      if (!(servos[i]["max_rad"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_ERROR("Invalid/Missing max_rad for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.max_rad = static_cast<double>(servos[i]["max_rad"]);
      }

      if (!(servos[i]["encoder_resolution"].getType() == XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR("Invalid/Missing encoder_resolution for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.encoder_resolution = static_cast<int>(servos[i]["encoder_resolution"]);
      }

      if (!(servos[i]["raw_speed_multiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_ERROR("Invalid/Missing raw_speed_multiplier for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.raw_speed_multiplier = static_cast<double>(servos[i]["raw_speed_multiplier"]);
      }

      if (!(servos[i]["angular_range"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_ERROR("Invalid/Missing angular_range for servo index %d, id=%d", i, info.id);
        ROS_BREAK();
      }
      else
      {
        info.angular_range = static_cast<double>(servos[i]["angular_range"]);
      }

      if (!(servos[i]["min_m"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        if (info.joint_type == "prismatic")
        {
          ROS_ERROR("Invalid/Missing min_m for servo index %d, id=%d", i, info.id);
          ROS_BREAK();
        }
        else
        {
          info.min_m = 0.0;
        }
      }
      else
      {
        info.min_m = static_cast<double>(servos[i]["min_m"]);
      }

      if (!(servos[i]["max_m"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        if (info.joint_type == "prismatic")
        {
          ROS_ERROR("Invalid/Missing max_m for servo index %d, id=%d", i, info.id);
          ROS_BREAK();
        }
        else
        {
          info.max_m = 0.0;
        }
      }
      else
      {
        info.max_m = static_cast<double>(servos[i]["max_m"]);
      }

      // Store an index if the axis needs limit switches, otherwise -1
      if (servos[i].hasMember("lim_sw_pos"))
      {
        if (!(servos[i]["lim_sw_pos"].getType() == XmlRpc::XmlRpcValue::TypeInt))
        {
          info.lim_sw_pos = -1;  // mark as index
        }
        else
        {
          info.lim_sw_pos = static_cast<int>(servos[i]["lim_sw_pos"]);
          ROS_INFO("lim_sw_pos=%d", info.lim_sw_pos);
          // bIODataNeeded_ = true;
        }
      }
      else
        info.lim_sw_pos = -1;

      if (servos[i].hasMember("lim_sw_neg"))
      {
        if (!(servos[i]["lim_sw_neg"].getType() == XmlRpc::XmlRpcValue::TypeInt))
        {
          info.lim_sw_neg = -1;  // mark as index
        }
        else
        {
          info.lim_sw_neg = static_cast<int>(servos[i]["lim_sw_neg"]);
          ROS_INFO("lim_sw_neg=%d", info.lim_sw_neg);
          // bIODataNeeded_ = true;
        }
      }
      else
        info.lim_sw_neg = -1;

      if (servos[i].hasMember("rest_position"))
      {
        if (!(servos[i]["rest_position"].getType() == XmlRpc::XmlRpcValue::TypeDouble))
        {
          info.rest_position = std::nan("");  // mark as index
        }
        else
        {
          info.rest_position = static_cast<double>(servos[i]["rest_position"]);
          ROS_INFO("rest_position=%f", info.rest_position);
          // bIODataNeeded_ = true;
        }
      }
      else
        info.rest_position = std::nan("");  // mark as index

      if (servos[i].hasMember("invert_direction"))
      {
        if (!(servos[i]["invert_direction"].getType() == XmlRpc::XmlRpcValue::TypeBoolean))
        {
          info.invert_direction = false;  // mark as index
        }
        else
        {
          info.invert_direction = static_cast<bool>(servos[i]["invert_direction"]);
          ROS_INFO("invert_direction=%d", info.invert_direction);
          // bIODataNeeded_ = true;
        }
      }
      else
        info.invert_direction = false;
      if (servos[i].hasMember("send_to_rest_position_at_end"))
      {
        if (!(servos[i]["send_to_rest_position_at_end"].getType() == XmlRpc::XmlRpcValue::TypeBoolean))
        {
          info.send_to_rest_position_at_end = false;  // mark as index
        }
        else
        {
          info.send_to_rest_position_at_end = static_cast<bool>(servos[i]["send_to_rest_position_at_end"]);
          ROS_INFO("send_to_rest_position_at_end=%d", info.send_to_rest_position_at_end);
          // bIODataNeeded_ = true;
        }
      }
      else
        info.send_to_rest_position_at_end = false;

      // Store in the std associative map of motor data
      ROS_INFO("joint_name = %s loaded", info.joint_name.c_str());
      // joint2dynamixel_[info.joint_name] = info;
      info_dynamixel[info.id] = info;
      joint_names_.push_back(info.joint_name);
      joint2id_[info.joint_name] = info.id;
      id2controlmode_[info.id] = UNKNOWN_CONTROL_MODE;
    }  // for
  }
  else
  {
    ROS_ERROR("No servos details loaded to param server");
    ROS_BREAK();
  }
  return true;
}

void DProHWInterface::init()
{
  read_servo_parameters(info_dynamixel_);

  dynamixel_helper_ = new DynamixelHelper(dynamixel_port_, dynamixel_baudate_);
  dynamixel_helper_->setDynamixelInfos(info_dynamixel_);
  if (!dynamixel_helper_->connect())
  {
    ROS_ERROR_STREAM("connect" << dynamixel_helper_->getLastMessageError());
    return;
  }

  hardwareinterface2controlmode_["hardware_interface::PositionJointInterface"] = POSITION_CONTROL;
  hardwareinterface2controlmode_["hardware_interface::VelocityJointInterface"] = VELOCITY_CONTROL;
  hardwareinterface2controlmode_["hardware_interface::EffortJointInterface"] = TORQUE_CONTROL;

  // Call parent class version of this function
  num_joints_ = joint_names_.size();

  // Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Command
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);
  previous_joint_position_command_.resize(num_joints_, std::nan(""));
  previous_joint_velocity_command_.resize(num_joints_, std::nan(""));
  previous_joint_effort_command_.resize(num_joints_, std::nan(""));

  // Limits
  joint_position_lower_limits_.resize(num_joints_, 0.0);
  joint_position_upper_limits_.resize(num_joints_, 0.0);
  joint_velocity_limits_.resize(num_joints_, 0.0);
  joint_effort_limits_.resize(num_joints_, 0.0);

  // Initialize interfaces for each joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

    // Add command interfaces to joints
    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);

    position_joint_interface_.registerHandle(joint_handle_position);

    hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);

    velocity_joint_interface_.registerHandle(joint_handle_velocity);

    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);

    effort_joint_interface_.registerHandle(joint_handle_effort);

    // Load the joint limits
    registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
  }  // end for each joint

  registerInterface(&joint_state_interface_);     // From RobotHW base class.
  registerInterface(&position_joint_interface_);  // From RobotHW base class.
  registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
  registerInterface(&effort_joint_interface_);    // From RobotHW base class.


  ROS_INFO_NAMED(name_, "DProHWInterface Ready.");
}

bool DProHWInterface::canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                const std::list<hardware_interface::ControllerInfo>& stop_list) const
{
  // nothing to check
  return true;
}

void DProHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  for (auto ci : stop_list)
  {
    for (auto claimed_resource : ci.claimed_resources)
    {
      for (auto resource : claimed_resource.resources)
      {
        if (joint2id_.find(resource) != joint2id_.end())
        {
          if (!dynamixel_helper_->setTorqueEnabled(joint2id_[resource], 0))
          {
            ROS_ERROR_STREAM("stop: " << dynamixel_helper_->getLastMessageError());
            return;
          }
          id2controlmode_[joint2id_[resource]] = UNKNOWN_CONTROL_MODE;
          // ROS_INFO_STREAM("Stopped controller: " << ci.name << " of type " << claimed_resource.hardware_interface
          //                                        << " which uses joint " << resource);
        }
      }
    }
  }

  for (auto ci : start_list)
  {
    for (auto claimed_resource : ci.claimed_resources)
    {
      for (auto resource : claimed_resource.resources)
      {
        if (joint2id_.find(resource) != joint2id_.end())
        {
          int new_control_mode = hardwareinterface2controlmode_[claimed_resource.hardware_interface];
          if (!dynamixel_helper_->setOperatingMode(joint2id_[resource], new_control_mode))
          {  // XXX Check condition error
            ROS_ERROR_STREAM("start: error setting operating mode: " << dynamixel_helper_->getLastMessageError());
            id2controlmode_[joint2id_[resource]] = new_control_mode;
          }
          else
          {
            id2controlmode_[joint2id_[resource]] = new_control_mode;
            ROS_INFO_STREAM("start: setting control mode to: " << new_control_mode);
          }

          if (!dynamixel_helper_->setTorqueEnabled(joint2id_[resource], 0))
          {
            ROS_ERROR_STREAM("start: error unsetting torque enable: " << dynamixel_helper_->getLastMessageError());
          }

          if (!limitVelocityOfPositionControlledMotor(joint2id_[resource]))
          {
            ROS_ERROR_STREAM("start: error while limiting velocity of position controlled motor " /*<< id*/
                             << ": " << dynamixel_helper_->getLastMessageError());
          }

          if (!dynamixel_helper_->setTorqueEnabled(joint2id_[resource], 1))
          {
            ROS_ERROR_STREAM("start: error setting torque enable" << dynamixel_helper_->getLastMessageError());
          }
          ROS_INFO_STREAM("Started controller: " << ci.name << " of type " << claimed_resource.hardware_interface
                                                 << " for resource " << resource
                                                 << " with operating mode: " << new_control_mode);
        }
      }
    }
  }
}

void DProHWInterface::read(ros::Duration& elapsed_time)
{
  std::map<int, std::vector<double> > data;
  size_t single_data_size = 3;  // pos + vel + current
  for (size_t i = 0; i < num_joints_; i++)
  {
    int id = joint2id_[joint_names_[i]];
    data[id] = std::vector<double>(single_data_size);
  }
  if (!dynamixel_helper_->getMultiData(data))
  {
    ROS_ERROR_STREAM("Error while reading data from Dynamixels: " << dynamixel_helper_->getLastMessageError());
  }

  for (size_t i = 0; i < num_joints_; i++)
  {
    int id = joint2id_[joint_names_[i]];
    double direction = info_dynamixel_[id].invert_direction ? -1 : 1;
    joint_position_[i] = data[id][0] * direction;
    joint_velocity_[i] = data[id][1] * direction;
    // TODO: convert effort from data read
    joint_effort_[i] = data[id][2] * direction;
  }
}

void DProHWInterface::write(ros::Duration& elapsed_time)
{
  // ROS_INFO("write");
  //  enforceLimits(elapsed_time);

  std::map<int, std::vector<double> > commands_for_position_motors;
  std::map<int, std::vector<double> > commands_for_velocity_motors;

  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (joint2id_.find(joint_names_[i]) == joint2id_.end())
    {
      ROS_WARN_STREAM_ONCE("Joint " << joint_names_[i] << " is not being controlled by me");
      continue;
    }

    int id = joint2id_.at(joint_names_[i]);
    double direction = info_dynamixel_[id].invert_direction ? -1 : 1;

//    ROS_INFO("i %d name %s id %d control mode %d POS %d", i, joint_names_[i].c_str(), id, id2controlmode_[id],
//             POSITION_CONTROL);
    if (id2controlmode_[id] == POSITION_CONTROL)
    {
      if (joint_position_command_[i] != previous_joint_position_command_[i])
      {
        std::vector<double> command;
        command.push_back(joint_position_command_[i] * direction);
        commands_for_position_motors[id] = command;
      }
      previous_joint_position_command_[i] = joint_position_command_[i];
    }

    if (id2controlmode_[id] == VELOCITY_CONTROL)
    {
      if (joint_velocity_command_[i] != previous_joint_velocity_command_[i])
      {
        std::vector<double> command;
        command.push_back(joint_velocity_command_[i] * direction);
        commands_for_velocity_motors[id] = command;
      }
      previous_joint_velocity_command_[i] = joint_velocity_command_[i];
    }

    if (id2controlmode_[id] == TORQUE_CONTROL)
    {
      ROS_WARN_STREAM("Torque Control not implemented for joint id: " << id << " with name: " << joint_names_[i]);
    }
  }

  if (!dynamixel_helper_->setMultiPosition(commands_for_position_motors))
  {
    ROS_ERROR_STREAM("Error while sending positions to motors: " << dynamixel_helper_->getLastMessageError());
  }
  else
  {
    // for (auto c : commands_for_position_motors)
    //   ROS_INFO_STREAM("commands_for_position_motors: " << c.second[0]);
  }

  if (!dynamixel_helper_->setMultiVelocity(commands_for_velocity_motors))
  {
    ROS_ERROR_STREAM("Error while sending velocities to motors: " << dynamixel_helper_->getLastMessageError());
  }
}

bool DProHWInterface::sendPositionControlledMotorsToRestPosition()
{
  std::map<int, std::vector<double> > commands_for_position_motors;

  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (joint2id_.find(joint_names_[i]) == joint2id_.end())
    {
      ROS_WARN_STREAM_ONCE("Joint " << joint_names_[i] << " is not being controlled by me");
      continue;
    }

    int id = joint2id_.at(joint_names_[i]);
    if (id2controlmode_[id] == POSITION_CONTROL && !std::isnan(info_dynamixel_[id].rest_position)  && info_dynamixel_[id].send_to_rest_position_at_end)
    {
      ROS_WARN_STREAM_ONCE("Sending Joint " << joint_names_[i] << " to rest position");
      std::vector<double> command;
      command.push_back(info_dynamixel_[id].rest_position);
      commands_for_position_motors[id] = command;
    }
  }

  if (!dynamixel_helper_->setMultiPosition(commands_for_position_motors))
  {
    ROS_ERROR_STREAM("Error while sending positions to motors: " << dynamixel_helper_->getLastMessageError());
    return false;
  }

  return true;
}

bool DProHWInterface::limitVelocityOfPositionControlledMotor(int id)
{
  // in position control, velocity is limited to value stored at the velocity goal position of the control table
  // we do this once each time the control type is changed
  std::map<int, std::vector<double> > limited_vels;

  if (id2controlmode_[id] == POSITION_CONTROL)
  {
    std::vector<double> cmd;
    cmd.push_back(info_dynamixel_[id].max_vel_rads);  // TODO: check rads or value
    limited_vels[id] = cmd;
    if (!dynamixel_helper_->setMultiVelocity(limited_vels))
    {
      return false;
    }
  }
  return true;
}

void DProHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  vel_jnt_sat_interface_.enforceLimits(period);
  eff_jnt_sat_interface_.enforceLimits(period);

  pos_jnt_soft_limits_.enforceLimits(period);
  vel_jnt_soft_limits_.enforceLimits(period);
  eff_jnt_soft_limits_.enforceLimits(period);
}

void DProHWInterface::registerJointLimits(const hardware_interface::JointHandle& joint_handle_position,
                                          const hardware_interface::JointHandle& joint_handle_velocity,
                                          const hardware_interface::JointHandle& joint_handle_effort,
                                          std::size_t joint_id)
{
  // Default values
  joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
  joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

  // Limits datastructures
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  bool has_joint_limits = false;
  bool has_soft_limits = false;

  // Get limits from URDF
  if (urdf_model_ == NULL)
  {
    ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
    return;
  }

  // Get limits from URDF
  const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

  // Get main joint limits
  if (urdf_joint == NULL)
  {
    ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
    return;
  }

  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                           << joint_limits.min_position << ", " << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                             << joint_limits.max_velocity << "]");
  }
  else
  {
    if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF "
                                                                         "position limit");
  }

  // Get limits from ROS param
  if (use_rosparam_joint_limits_)
  {
    if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
    {
      has_joint_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has rosparam position limits ["
                                             << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      if (joint_limits.has_velocity_limits)
        /*ROS_DEBUG*/ ROS_INFO_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                                            << " has rosparam velocity limit ["
                                                            << joint_limits.max_velocity << "]");
    }  // the else debug message provided internally by joint_limits_interface
  }

  // Get soft limits from URDF
  if (use_soft_limits_if_available_)
  {
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
    {
      has_soft_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint "
                                                                          "limits");
    }
  }

  // Quit we we haven't found any limits in URDF or rosparam server
  if (!has_joint_limits)
  {
    return;
  }

  // Copy position limits if available
  if (joint_limits.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_limits.min_position += std::numeric_limits<double>::epsilon();
    joint_limits.max_position -= std::numeric_limits<double>::epsilon();

    joint_position_lower_limits_[joint_id] = joint_limits.min_position;
    joint_position_upper_limits_[joint_id] = joint_limits.max_position;
  }

  // Copy velocity limits if available
  if (joint_limits.has_velocity_limits)
  {
    joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
  }

  // Copy effort limits if available
  if (joint_limits.has_effort_limits)
  {
    joint_effort_limits_[joint_id] = joint_limits.max_effort;
  }

  if (has_soft_limits)  // Use soft limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");

    if (urdf_joint->type == urdf::Joint::REVOLUTE)
    {  // only register position interface for revolute joints
      const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
                                                                                       joint_limits, soft_limits);
      pos_jnt_soft_limits_.registerHandle(soft_handle_position);
    }
    if (urdf_joint->type == urdf::Joint::CONTINUOUS)
    {  // only register velocity interface for continous joints
      const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
                                                                                       joint_limits, soft_limits);
      vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
    }

    // register effort interfaces always
    const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
                                                                                 soft_limits);
    eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
  }
  else  // Use saturation limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

    if (urdf_joint->type == urdf::Joint::REVOLUTE)
    {  // only register position interface for revolute joints
      const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position,
                                                                                      joint_limits);
      pos_jnt_sat_interface_.registerHandle(sat_handle_position);
    }
    if (urdf_joint->type == urdf::Joint::CONTINUOUS)
    {  // only register velocity interface for continous joints
      const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity,
                                                                                      joint_limits);
      vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);
    }
    // register effort interfaces always
    const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
    eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
  }
}

void DProHWInterface::reset()
{
  // Reset joint limits state, in case of mode switch or e-stop
  pos_jnt_sat_interface_.reset();
  pos_jnt_soft_limits_.reset();
}

void DProHWInterface::printState()
{
  // WARNING: THIS IS NOT REALTIME SAFE
  // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
  ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
}

std::string DProHWInterface::printStateHelper()
{
  std::stringstream ss;
  std::cout.precision(15);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << joint_position_[i] << "\t ";
    ss << std::fixed << joint_velocity_[i] << "\t ";
    ss << std::fixed << joint_effort_[i] << std::endl;
  }
  return ss.str();
}

std::string DProHWInterface::printCommandHelper()
{
  std::stringstream ss;
  std::cout.precision(15);
  ss << "    position     velocity         effort  \n";
  for (size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << joint_position_command_[i] << "\t ";
    ss << std::fixed << joint_velocity_command_[i] << "\t ";
    ss << std::fixed << joint_effort_command_[i] << std::endl;
  }
  return ss.str();
}

void DProHWInterface::loadURDF(ros::NodeHandle& nh, std::string param_name)
{
  std::string urdf_string;
  urdf_model_ = new urdf::Model();

  urdf_string_ = "";
  // search and wait for robot_description on param server
  while (urdf_string_.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
      ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                  << search_param_name);
      nh.getParam(search_param_name, urdf_string_);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                  << param_name);
      nh.getParam(param_name, urdf_string_);
    }

    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string_))
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
  else
    ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
}

}  // namespace
