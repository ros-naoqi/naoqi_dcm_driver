#ifndef DCM_HPP
#define DCM_HPP

// NAOqi Headers
#include <qi/session.hpp>

/**
 * @brief This class is a wapper for Naoqi DCM Class
 */
class DCM
{
public:
  DCM(const qi::SessionPtr& session,
      const double &controller_freq);

  //! @brief initialize all Aliases
  bool init(const std::vector <std::string> &joints);

  //! @brief update joints values
  void writeJoints(const std::vector <double> &joint_commands);

  //! @brief update joints stiffness
  bool setStiffness(const float &stiffness);

  //! @brief get time from the DCM proxy
  int getTime(const int &offset);

private:
  //! @brief initialize of DCM Motion commands
  void createPositionActuatorCommand(const std::vector <std::string> &joints);

  //! @brief create Position Actuator Alias
  bool createPositionActuatorAlias(const std::vector <std::string> &joints);

  //! @brief create Hardness Actuator Alias
  bool createHardnessActuatorAlias(const std::vector <std::string> &joints);

  //! @brief DCM Wrapper Method
  bool DCMAliasTimedCommand(const std::string& alias,
                            const float& values,
                            const int& timeOffset,
                            const std::string& type_update="Merge");

  /** DCM proxy */
  qi::AnyObject dcm_proxy_;

  /** alias to send DCM commands */
  std::vector <qi::AnyValue> commands_;

  /** alias keys to send DCM commands */
  std::vector <std::vector <std::vector <qi::AnyValue> > > commands_values_;

  /** frequency to write joints values */
  double controller_freq_;
};
#endif // DCM_HPP
