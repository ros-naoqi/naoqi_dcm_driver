#ifndef MEMORY_HPP
#define MEMORY_HPP

// NAOqi Headers
#include <qi/session.hpp>

/**
 * @brief This class is a wapper for Naoqi Memory Class
 */
class Memory
{
public:
  Memory(const qi::SessionPtr& session);

  //! @brief initialize with joints names to control
  void init(const std::vector <std::string> &joints_names);

  //! @brief initialize memory keys to read
  std::vector <std::string> initMemoryKeys(const std::vector <std::string> &joints);

  //! @brief Get values of keys
  std::vector<float> getListData();

  //! @brief Get values associated with the given list of keys
  std::vector<float> getListData(const std::vector <std::string> &keys);

  //! @brief get a key-value pair stored in memory
  std::string getData(const std::string &str);

  //! @brief subscribe to a micro-event
  void subscribeToMicroEvent(const std::string &name,
                             const std::string &callback_module,
                             const std::string &callback_method,
                             const std::string &callback_message);

  //! @brief unsubscribe from a micro-event
  void unsubscribeFromMicroEvent(const std::string &name,
                                 const std::string &callback_module);

private:
  /** Memory proxy */
  qi::AnyObject memory_proxy_;

  /** joints positions keys to read */
  std::vector <std::string> keys_positions_;
};

#endif // MEMORY_HPP
