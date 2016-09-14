#ifndef MEMORY_HPP
#define MEMORY_HPP

// NAOqi Headers
#include <qi/session.hpp>

/**
 * @brief This class is a wapper for Naoqi Memory Proxy
 */
class Memory
{
public:
  Memory(const qi::SessionPtr& session);

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
};

#endif // MEMORY_HPP
