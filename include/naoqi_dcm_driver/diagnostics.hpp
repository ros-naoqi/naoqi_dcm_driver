#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

// NAOqi Headers
#include <qi/session.hpp>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

/**
 * @brief This class defines a Diagnostic
 * It is used to check the robot state and sent to requesting nodes
 */
class Diagnostics
{
public:
  /**
  * @brief Constructor
  * @param session[in] Naoqi session
  * @param pub[in] ROS topic publisher
  * @param joints_all_names[in] all joints to check
  * @param robot[in] robot type
  */
  Diagnostics(const qi::SessionPtr& session,
              ros::Publisher *pub,
              const std::vector<std::string> &joints_all_names,
              const std::string &robot);

  //! @brief destroys all ros nodehandle and shutsdown all publisher
  virtual ~Diagnostics() {}

  //! @brief publish the newly received data
  bool publish();

  //! @brief set the message based on level
  void setMessageFromStatus(diagnostic_updater::DiagnosticStatusWrapper &status);

  //! @brief set the aggregated message
  void setAggregatedMessage(diagnostic_updater::DiagnosticStatusWrapper &status);

  //! @brief return the status message
  std::string getStatusMsg();

private:
  /** diagnostics publisher */
  ros::Publisher *pub_;

  /** Memory proxy */
  qi::AnyObject memory_proxy_;

  /** joints names */
  std::vector <std::string> joints_all_names_;

  /** all the keys to check. It is a concatenation of
   * temperatures_keys, stiffness_keys, current_keys */
  std::vector <std::string> keys_tocheck_;

  /** the temperature to alert a warning */
  float temperature_warn_level_;

  /** the temperature to alert an error */
  float temperature_error_level_;

  /** The status message */
  diagnostic_updater::DiagnosticStatusWrapper status_;
};

#endif // DIAGNOSTICS_H
