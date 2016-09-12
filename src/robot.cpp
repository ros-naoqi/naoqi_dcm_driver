/*
 * Copyright 2016 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <sensor_msgs/JointState.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include "naoqi_dcm_driver/robot.hpp"
#include "naoqi_dcm_driver/tools.hpp"

QI_REGISTER_OBJECT( Robot,
                    isConnected,
                    connect,
                    stopService);

Robot::Robot(qi::SessionPtr session):
               _session(session),
               session_name_("naoqi_dcm_driver"),
               is_connected_(false),
               nhPtr_(new ros::NodeHandle("")),
               body_type_(""),
               topic_queue_(10),
               prefix_("naoqi_dcm"),
               high_freq_(50.0),
               controller_freq_(15.0),
               joint_precision_(0.1),
               odom_frame_("odom"),
               temperature_error_(70.0f)
{
}

Robot::~Robot()
{
  ROS_INFO_STREAM(session_name_ << " service is shutting down.");
  stopService();
}

void Robot::stopService() {
  ROS_INFO_STREAM("Stopping the service...");

  //going to rest
  //set stiffness after wakeUp the robot
  try
  {
    motion_proxy_.call<void>("stiffnessInterpolation", "LArm", 0.0f, 1.0f);
    motion_proxy_.call<void>("stiffnessInterpolation", "RArm", 0.0f, 1.0f);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Could not WakeUp the robot : %s", e.what());
  }
  if (motor_groups_.size() == 1)
    if (motor_groups_[0] == "Body")
      if (motion_proxy_.call<bool>("robotIsWakeUp"))
      {
        motion_proxy_.call<void>("rest");
        ROS_INFO_STREAM("Going to rest ...");
        sleep(4);
      }

  //reset stiffness
  setStiffness(0.0f);
  is_connected_ = false;

  if(nhPtr_)
  {
    nhPtr_->shutdown();
    ros::shutdown();
  }
}

bool Robot::initialize()
{
  //set alias for Memory keys to check
  for(std::vector<std::string>::iterator it=joints_names_.begin(); it!=joints_names_.end(); ++it)
  {
    //ignore mimic joints
    if( (it->find("Wheel") != std::string::npos)
         || (*it=="RHand" || *it=="LHand" || *it == "RWristYaw" || *it == "LWristYaw") && (body_type_ == "H21"))
    {
      joints_names_.erase(it);
      it--;
      continue;
    }
    keys_positions_.push_back("Device/SubDeviceList/" + *it + "/Position/Sensor/Value");
  }

  //keet the number of active joints
  int joints_nbr = joints_names_.size();

  // DCM Motion Commands Initialization
  // Create the Motion Command
  // at first, set keys
  commands_values_.reserve(joints_nbr);
  commands_values_.resize(joints_nbr);
  for(int i=0; i<joints_nbr; ++i)
  {
    commands_values_[i].resize(1);
    commands_values_[i][0].resize(3);
    commands_values_[i][0][2] = qi::AnyValue(qi::AnyReference::from(0), false, false);
  }
  // then, set the alias
  commands_.reserve(4);
  commands_.resize(4);
  commands_[0] = qi::AnyValue(qi::AnyReference::from("Joints"), false, false);
  commands_[1] = qi::AnyValue(qi::AnyReference::from("ClearAll"), false, false);
  commands_[2] = qi::AnyValue(qi::AnyReference::from("time-mixed"), false, false);
  commands_[3] = qi::AnyValue(qi::AnyReference::from(commands_values_), false, false);

  // Create the Joints Actuators Alias
  //at first, set keys
  std::vector <qi::AnyValue> commandAlias_j_keys;
  commandAlias_j_keys.resize(joints_nbr);
  for(int i=0; i<joints_nbr; ++i)
  {
    std::string key = "Device/SubDeviceList/" + joints_names_.at(i) + "/Position/Actuator/Value";
    commandAlias_j_keys[i] = qi::AnyValue(qi::AnyReference::from(key), false, false);
  }
  // then, set the alias
  std::vector <qi::AnyValue> commandAlias_j;
  commandAlias_j.resize(2);
  commandAlias_j[0] = qi::AnyValue(qi::AnyReference::from("Joints"), false, false);
  commandAlias_j[1] = qi::AnyValue(qi::AnyReference::from(commandAlias_j_keys), false, false);

  try
  {
    qi::AnyValue commandAlias_qi(qi::AnyReference::from(commandAlias_j), false, false);
    dcm_proxy_.call<void>("createAlias", commandAlias_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not initialize DCM aliases for position actuators !\n\tTrace: %s", e.what());
    return false;
  }

  // Create Joints Hardness Alias
  //at first, set keys
  std::vector <qi::AnyValue> commandAlias_h_keys;
  for(int i=0; i<joints_nbr; ++i)
  {
    if((joints_names_.at(i) == "RHipYawPitch") //for mimic joints: Nao only
        || (joints_names_.at(i).find("Wheel") != std::string::npos))
      continue;

    std::string key = "Device/SubDeviceList/" + joints_names_.at(i) + "/Hardness/Actuator/Value";
    commandAlias_h_keys.push_back(qi::AnyValue(qi::AnyReference::from(key), false, false));
  }
  // then, set the alias
  std::vector <qi::AnyValue> commandAlias_h;
  commandAlias_h.reserve(2);
  commandAlias_h.resize(2);
  commandAlias_h[0] = qi::AnyValue(qi::AnyReference::from("JointsHardness"), false, false);
  commandAlias_h[1] = qi::AnyValue(qi::AnyReference::from(commandAlias_h_keys), false, false);

  //call dcm_proxy.createAlias
  try
  {
    qi::AnyValue commandAlias_qi(qi::AnyReference::from(commandAlias_h), false, false);
    dcm_proxy_.call<void>("createAlias", commandAlias_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not initialize DCM aliases for joints hardness!\n\tTrace: %s", e.what());
    return false;
  }

  // Turn Stiffness On
  setStiffness(1.0f);

  return true;
}

bool Robot::initializeControllers()
{
  if(!initialize())
  {
    ROS_ERROR("Initialization method failed!");
    return false;
  }
  int joints_nbr = joints_names_.size();

  // Initialize Controllers' Interfaces
  joint_angles_.reserve(joints_nbr);
  joint_velocities_.reserve(joints_nbr);
  joint_efforts_.reserve(joints_nbr);
  joint_commands_.reserve(joints_nbr);

  joint_angles_.resize(joints_nbr);
  joint_velocities_.resize(joints_nbr);
  joint_efforts_.resize(joints_nbr);
  joint_commands_.resize(joints_nbr);

  try
  {
    for(int i=0; i<joints_nbr; ++i)
    {
      hardware_interface::JointStateHandle state_handle(joints_names_.at(i), &joint_angles_[i],
                                                        &joint_velocities_[i], &joint_efforts_[i]);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joints_names_.at(i)),
                                                 &joint_commands_[i]);
      jnt_pos_interface_.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);
  }
  catch(const ros::Exception& e)
  {
    ROS_ERROR("Could not initialize hardware interfaces!\n\tTrace: %s", e.what());
    return false;
  }
  ROS_INFO_STREAM(session_name_ << " module initialized!");
  return true;
}


// The entry point from outside
bool Robot::connect()
{
  is_connected_ = false;

  // Load ROS Parameters
  loadParams();

  // Initialize DCM Proxy
  try
  {
    dcm_proxy_ = _session->service("DCM");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to connect to DCM Proxy!\n\tTrace: %s", e.what());
    return false;
  }

  // Initialize Memory Proxy
  try
  {
    memory_proxy_ = _session->service("ALMemory");

    //get the robot name
    robot_ = memory_proxy_.call<std::string>("getData", "RobotConfig/Body/Type" );
    std::transform(robot_.begin(), robot_.end(), robot_.begin(), ::tolower);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to connect to Memory Proxy!\n\tTrace: %s", e.what());
    return false;
  }

  // Allow for temperature reporting (for CPU)
  try
  {
    if ((robot_ == "pepper") || (robot_ == "nao")) {
      qi::AnyObject body_temperature_ = _session->service("ALBodyTemperature");
      body_temperature_.call<void>("setEnableNotifications", true);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to connect to ALBodyTemperature!\n\tTrace: %s", e.what());
    return false;
  }

  // Initialize Motion Proxy
  try
  {
    motion_proxy_ = _session->service("ALMotion");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to connect to Motion Proxy!\n\tTrace: %s", e.what());
    return false;
  }

  // Initialize joints to control
  try
  {
    //going to rest
    if (!motion_proxy_.call<bool>("robotIsWakeUp"))
    {
      if (motor_groups_.size() > 1)
      {
        ROS_ERROR("Please, wakeUp the robot to be able to set stiffness");
        stopService();
      }
      if (motor_groups_.size() == 1)
        if (motor_groups_[0] == "Body")
        {
          motion_proxy_.call<void>("wakeUp");
          ROS_INFO_STREAM("Going to wakeup ...");
          sleep(4);
        }
    }

    //reading joints names that will be controlled
    try
    {
      for (std::vector<std::string>::const_iterator it=motor_groups_.begin(); it!=motor_groups_.end(); ++it)
      {
        std::vector <std::string> joint_names_g =
            motion_proxy_.call<std::vector <std::string> >("getBodyNames", *it);
        joints_names_.insert(joints_names_.end(), joint_names_g.begin(), joint_names_g.end());
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Failed to getBodyNames!\n\tTrace: %s", e.what());
    }
    std::stringstream ss;
    std::copy(joints_names_.begin(), joints_names_.end()-1, std::ostream_iterator<std::string>(ss,", "));
    std::copy(joints_names_.end()-1, joints_names_.end(), std::ostream_iterator<std::string>(ss));
    ROS_INFO("Robot's joints that will be controlled are: %s",ss.str().c_str());

    //joint_states topic initialization
    joint_states_topic_.name =
        motion_proxy_.call<std::vector <std::string> >("getBodyNames", "Body");
    joint_states_topic_.position.resize(joint_states_topic_.name.size());

    //initializing the Diagnostics
    try
    {
      std::vector<std::string> joints_all_names =
          motion_proxy_.call<std::vector<std::string> >("getBodyNames", "JointActuators");
      diagPtr_ = boost::shared_ptr<Diagnostics>(
            new Diagnostics(_session, &diag_pub_, joints_all_names, temperature_error_));
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Failed to getBodyNames!\n\tTrace: %s", e.what());
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to getBodyNames from Motion proxy!\n\tTrace: %s", e.what());
    return false;
  }

  is_connected_ = true;

  // Subscribe/Publish ROS Topics/Services
  subscribe();

  // Initialize Controller Manager and Controllers
  manager_ = new controller_manager::ControllerManager( this, *nhPtr_);
  if(!initializeControllers())
  {
    ROS_ERROR("Could not load controllers!");
    return false;
  }
  ROS_INFO("Controllers successfully loaded!");
  return true;
}

void Robot::disconnect()
{
  if(!is_connected_)
    return;

  setStiffness(0.0f);

  try
  {
    unsubscribeFromMicroEvent("ClientDisconnected", session_name_);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to unsubscribe from subscribed events!\n\tTrace: %s", e.what());
  }
  is_connected_ = false;
}

void Robot::subscribe()
{
  // Subscribe/Publish ROS Topics/Services
  cmd_vel_sub_ = nhPtr_->subscribe(prefix_+"cmd_vel", topic_queue_, &Robot::commandVelocity, this);

  diag_pub_ = nhPtr_->advertise<diagnostic_msgs::DiagnosticArray>(prefix_+"diagnostics", topic_queue_);

  stiffness_pub_ = nhPtr_->advertise<std_msgs::Float32>(prefix_+"stiffnesses", topic_queue_);
  stiffness_.data = 1.0f;

  joint_states_pub_ = nhPtr_->advertise<sensor_msgs::JointState>(prefix_+"joint_states", topic_queue_);
}

void Robot::loadParams()
{
  ros::NodeHandle nh("~");
  // Load Server Parameters
  nh.getParam("BodyType", body_type_);
  nh.getParam("TopicQueue", topic_queue_);
  nh.getParam("Prefix", prefix_);
  prefix_ += "/";
  nh.getParam("HighCommunicationFrequency", high_freq_);
  nh.getParam("ControllerFrequency", controller_freq_);
  nh.getParam("JointPrecision", joint_precision_);
  nh.getParam("OdomFrame", odom_frame_);

  //FIXME: set it from controllers
  //choose motor groups to control
  std::string tmp="";
  nh.getParam("motor_groups", tmp);
  boost::erase_all(tmp, " ");
  boost::split (motor_groups_, tmp, boost::is_any_of(","));
  if (motor_groups_.size() == 1)
    if (motor_groups_[0].empty())
      motor_groups_.erase(motor_groups_.begin());
  if (motor_groups_.size() == 0)
  {
    motor_groups_.push_back("LArm");
    motor_groups_.push_back("RArm");
  }
}

void Robot::DCMAliasTimedCommand(const std::string& alias,
                                 const std::vector<float> &values,
                                 const std::vector<int> &timeOffsets,
                                 const std::string& type_update,
                                 const std::string& type_time)
{
  // Create Alias timed-command
  std::vector <qi::AnyValue> command;
  command.reserve(4);
  command.resize(4);
  command[0] = qi::AnyValue(qi::AnyReference::from(alias), false, false);
  command[1] = qi::AnyValue(qi::AnyReference::from(type_update), false, false);
  command[2] = qi::AnyValue(qi::AnyReference::from(type_time), false, false);

  std::vector <std::vector <std::vector <qi::AnyValue> > > command_keys;
  command_keys.resize(values.size());

  //get DCM time in 1 sec
  int time = dcm_proxy_.call<int>("getTime", 0);
  for(int i=0; i<values.size(); ++i)
  {
    command_keys[i].resize(1);
    command_keys[i][0].resize(2);
    command_keys[i][0][0] = qi::AnyValue(qi::AnyReference::from(values[i]), false, false);
    command_keys[i][0][1] = qi::AnyValue(qi::AnyReference::from(time+timeOffsets[i]), false, false);
  }
  command[3] = qi::AnyValue(qi::AnyReference::from(command_keys), false, false);
  qi::AnyValue command_qi = qi::AnyValue(qi::AnyReference::from(command), false, false);

  try
  {
    // Execute Alias timed-command
    dcm_proxy_.call<void>("setAlias", command_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not execute DCM timed-command!\n\t%s\n\n\tTrace: %s", alias.c_str(), e.what());
  }
}

void Robot::subscribeToMicroEvent(const std::string &name, const std::string &callback_module,
                                const std::string &callback_method, const std::string &callback_message)
{
  try
  {
    memory_proxy_.call<int>("subscribeToMicroEvent", name, callback_module, callback_message, callback_method);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not subscribe to micro-event '%s'.\n\tTrace: %s", name.c_str(), e.what());
  }
}

void Robot::unsubscribeFromMicroEvent(const std::string &name, const std::string &callback_module)
{
  try
  {
    memory_proxy_.call<void>("unsubscribeToMicroEvent", name, callback_module);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not unsubscribe from micro-event '%s'.\n\tTrace: %s", name.c_str(), e.what());
  }
}

void Robot::run()
{
  controllerLoop();
}

void Robot::controllerLoop()
{
  static ros::Rate rate(controller_freq_);
  while(ros::ok())
  {
    ros::Time time = ros::Time::now();

    if(!is_connected_)
      break;

    try
    {
      dcm_proxy_.call<void>("ping");
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Could not ping DCM proxy.\n\tTrace: %s", e.what());
      is_connected_ = false;
      rate.sleep();
      continue;
    }

    //publishBaseFootprint(time);

    if(stiffnesses_enabled_)
    {
      stiffness_.data = 1.0f;
    }
    else
    {
      stiffness_.data = 0.0f;
    }
    stiffness_pub_.publish(stiffness_);

    readJoints();

    manager_->update(time, ros::Duration(1.0f/controller_freq_));

    stiffness_pub_.publish(stiffness_);

    writeJoints();

    //publishJointStateFromAlMotion();

    rate.sleep();
  }
  ROS_INFO_STREAM("Shutting down the main loop");
}

bool Robot::isConnected()
{
  return is_connected_;
}

void Robot::commandVelocity(const geometry_msgs::TwistConstPtr &msg)
{
  // no need to check for max velocity since motion clamps the velocities internally
  const float& vel_x = msg->linear.x;
  const float& vel_y = msg->linear.y;
  const float& vel_th = msg->angular.z;
  std::cout << "going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th << std::endl;
  motion_proxy_.async<void>("move", vel_x, vel_y, vel_th);
}

void Robot::publishBaseFootprint(const ros::Time &ts)
{
  std::string odom_frame, base_link_frame;
  try 
  {
    odom_frame = base_footprint_listener_.resolve(odom_frame_);
    base_link_frame = base_footprint_listener_.resolve("base_link");
  }
  catch(ros::Exception& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }

  tf::StampedTransform tf_odom_to_base, tf_odom_to_left_foot, tf_odom_to_right_foot;
  double temp_freq = 1.0f/(10.0*high_freq_);
  if(!base_footprint_listener_.waitForTransform(odom_frame, base_link_frame, ros::Time(0), ros::Duration(temp_freq)))
    return;
  try 
  {
    base_footprint_listener_.lookupTransform(odom_frame, base_link_frame,  ros::Time(0), tf_odom_to_base);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("%s",ex.what());
    return ;
  }

  tf::Vector3 new_origin = (tf_odom_to_right_foot.getOrigin() + tf_odom_to_left_foot.getOrigin())/2.0;
  double height = std::min(tf_odom_to_left_foot.getOrigin().getZ(), tf_odom_to_right_foot.getOrigin().getZ());
  new_origin.setZ(height);

  double roll, pitch, yaw;
  tf_odom_to_base.getBasis().getRPY(roll, pitch, yaw);

  tf::Transform tf_odom_to_footprint(tf::createQuaternionFromYaw(yaw), new_origin);
  tf::Transform tf_base_to_footprint = tf_odom_to_base.inverse() * tf_odom_to_footprint;

  base_footprint_broadcaster_.sendTransform(tf::StampedTransform(tf_base_to_footprint, ts,
                                                                 base_link_frame, "base_footprint"));
}

void Robot::readJoints()
{
  //get joints positions
  std::vector<float> joint_positions;
  try
  {
    qi::AnyValue keys_positions_qi = memory_proxy_.call<qi::AnyValue>("getListData", keys_positions_);
    fromAnyValueToFloatVector(keys_positions_qi, joint_positions);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not get joint data from the robot \n\tTrace: %s", e.what());
    return;
  }

  if (!diagPtr_->publish())
  {
    setStiffness(0.0f);
    ROS_INFO_STREAM("HIGH JOINT TEMPERATURE IS DETECTED ");
    stopService();
  }

  std::vector <double>::iterator it_comm = joint_commands_.begin();
  std::vector <double>::iterator it_cur = joint_angles_.begin();
  std::vector <float>::iterator it_mem = joint_positions.begin();
  for(; it_comm<joint_commands_.end(); ++it_comm, ++it_cur, ++it_mem)
  {
    *it_cur = *it_mem;
    // Set commands to the read angles for when no command specified
    *it_comm = *it_mem;
  }
}

void Robot::publishJointStateFromAlMotion(){
  std::vector<double> positionData;
  positionData = motion_proxy_.call<std::vector <double> >("getAngles", "Body", 1);
  joint_states_topic_.header.stamp = ros::Time::now();
  joint_states_topic_.header.frame_id = "base_link";
  for(int i = 0; i<positionData.size(); ++i)
  {
     joint_states_topic_.position[i] = positionData[i];
  }

  joint_states_pub_.publish(joint_states_topic_);
}

void Robot::writeJoints()
{
  // Update joints only when actual command is issued
  bool changed(false);
  std::vector<double>::iterator it_now = joint_angles_.begin();
  std::vector<double>::iterator it_comm = joint_commands_.begin();
  for(int i=0; it_comm != joint_commands_.end(); ++i, ++it_comm, ++it_now)
  {
    double diff = fabs(*it_comm - *it_now);
    if(diff > joint_precision_)
    {
      //ROS_INFO_STREAM(" joint " << i << " : " << *it_now << " != " << *it_comm << " diff=" << diff);
      changed = true;
      break;
    }
  }
  // Do not write joints if no change in joint values
  if(!changed)
    return;

  try
  {
    int time = dcm_proxy_.call<int>("getTime", 0) + static_cast<int>(5000.0/controller_freq_);

    std::vector<double>::iterator it_comm = joint_commands_.begin();
    for(int i=0; i<joint_commands_.size(); ++i, ++it_comm)
    {
      commands_values_[i][0][0] = qi::AnyValue(qi::AnyReference::from(static_cast<float>(*it_comm)), false, false);
      commands_values_[i][0][1] = qi::AnyValue(qi::AnyReference::from(time), false, false);
    }

    commands_[3] = qi::AnyValue(qi::AnyReference::from(commands_values_), false, false);
    qi::AnyValue commands_qi = qi::AnyValue(qi::AnyReference::from(commands_), false, false);

    dcm_proxy_.call<void>("setAlias", commands_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Could not send joint commands to the robot : \n\tTrace: %s", e.what());
    return;
  }
}

bool Robot::setStiffness(const float &stiffness)
{
  //set stiffness after wakeUp the robot
  try
  {
    for (std::vector<std::string>::const_iterator it=motor_groups_.begin(); it!=motor_groups_.end(); ++it)
      motion_proxy_.call<void>("stiffnessInterpolation", *it, stiffness, 1.0f);
  }
  catch (const std::exception &e)
  {
     ROS_ERROR("Could not WakeUp the robot : %s", e.what());
     return false;
  }

//  replace by calling DCM
//  DCMAliasTimedCommand("JointsHardness", std::vector<float>(hardness_times_.size(), stiffness), hardness_times_);

  if (stiffness == 1.0f)
    stiffnesses_enabled_ = true;
  else
    stiffnesses_enabled_ = false;

  return true;
}
