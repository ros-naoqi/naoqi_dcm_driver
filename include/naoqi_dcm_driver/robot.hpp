#ifndef NAOQI_DCM_DRIVER_H
#define NAOQI_DCM_DRIVER_H

// Boost Headers
#include <boost/shared_ptr.hpp>

// NAOqi Headers
#include <qi/session.hpp>
#include <qi/anyobject.hpp>
#include <qi/os.hpp>
#include <qi/anyvalue.hpp>

// ROS Headers
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include "naoqi_dcm_driver/diagnostics.hpp"
#include "naoqi_dcm_driver/memory.hpp"
#include "naoqi_dcm_driver/dcm.hpp"
#include "naoqi_dcm_driver/motion.hpp"

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

class Robot : public hardware_interface::RobotHW
{
public:
  /**
  * @brief Constructor
  * @param session[in] session pointer for the service registration
  */
  Robot(qi::SessionPtr session);

  //! @brief destroys all ros nodehandle and shutsdown all publisher
  ~Robot();

  //! @brief Function to stop the service
  void stopService();

  //! @brief Function to check if service is connected
  bool isConnected();

  //! @brief Connection to ALProxies
  bool connect();

  //! @brief initialization of controllers based on joints names
  bool initializeControllers(const std::vector <std::string> &joints_names);

  //! @brief defining Subscribe/Advertise to ROS Topics/Services
  void subscribe();

  //! @brief loading parameters
  void loadParams();

  //! @brief the main loop
  void controllerLoop();

  //! @brief controlling the robot's velocity
  void commandVelocity(const geometry_msgs::TwistConstPtr &msg);

  //! @brief publishing the base_footprint
  void publishBaseFootprint(const ros::Time &ts);

  //! @brief reading joints values
  void readJoints();

  //! @brief publishing joint states
  void publishJointStateFromAlMotion();

  //! @brief set joints values
  void writeJoints();

  //! @brief set stiffness
  bool setStiffness(const float &stiffness);

  //! @brief start the main loop
  void run();

  //! @brief ignore mimic joints from control
  void ignoreMimicJoints(std::vector <std::string> *joints);

private:
  /** node handle pointer*/
  boost::scoped_ptr <ros::NodeHandle> nhPtr_;

  /** pointer to Diagnostics class */
  boost::shared_ptr <Diagnostics> diagnostics_;

  /** pointer to Memory class */
  boost::shared_ptr <Memory> memory_;

  /** pointer to DCM class */
  boost::shared_ptr <DCM> dcm_;

  /** pointer to Motion class */
  boost::shared_ptr <Motion> motion_;

  /** velocity publisher */
  ros::Subscriber cmd_vel_sub_;

  /** base_footprint broadcaster */
  tf::TransformBroadcaster base_footprint_broadcaster_;

  /** base_footprint listener */
  tf::TransformListener base_footprint_listener_;

  /** stiffness publisher */
  ros::Publisher stiffness_pub_;

  /** diagnostics publisher */
  ros::Publisher diag_pub_;

  /** joint states publisher */
  ros::Publisher joint_states_pub_;

  /** stiffness data */
  std_msgs::Float32 stiffness_;

  /** joint states data */
  sensor_msgs::JointState joint_states_topic_;

  /** controller manager */
  controller_manager::ControllerManager* manager_;

  /** service name */
  std::string session_name_;

  /** session connection status */
  bool is_connected_;

  /** robot body type */
  std::string body_type_;

  /** prefix for published topics */
  std::string prefix_;

  /** odom frame name */
  std::string odom_frame_;

  /** message buffer */
  int topic_queue_;

  /** frequency to read joints values */
  double high_freq_;

  /** frequency to write joints values */
  double controller_freq_;

  /** threshold to update joints to desired values */
  double joint_precision_;

  /** Naoqi session pointer */
  qi::SessionPtr _session;

  /** motor groups used to control */
  std::vector <std::string> motor_groups_;

  /** all joints positions keys to read */
  std::vector <std::string> keys_positions_all_;

  /** joints states from ROS hardware interface */
  hardware_interface::JointStateInterface jnt_state_interface_;

  /** joints positions from ROS hardware interface */
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  /** joints angles to apply */
  std::vector <double> joint_commands_;

  /** current joints angles */
  std::vector <double> joint_angles_;

  /** joints velocities from ROS hardware interface */
  std::vector <double> joint_velocities_;

  /** joints efforts from ROS hardware interface */
  std::vector <double> joint_efforts_;

  /** enable using DCM instead of ALMotion */
  bool use_dcm_;

  /** enable velocity control and publish cmd_vel */
  bool use_cmd_vel_;
};

#endif // NAOQI_DCM_DRIVER_H
