#ifndef RECONFIGURE_HANDLER_HPP
#define RECONFIGURE_HANDLER_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

namespace ros_util {

/**
 * @brief A Handler Class for the ROS Reconfigure server.
 *
 * @tparam T A Reconfigure Configuration Type
 */
template<typename T> class ReconfigureHandler
{
public:
  /**
   * @brief Construct a new Reconfigure Handler object
   *
   * @param t_defaultConfig Default reconfigure parameter object
   * @param configName Name of the reconfigure server configuration
   */
  ReconfigureHandler(T t_defaultConfig, std::string configName)
    : m_configServer(m_configMutex, ros::NodeHandle(configName))
  {
    m_configServer.updateConfig(std::move(t_defaultConfig)); //zasto ova linija??
    auto paramCallback = boost::bind(&ReconfigureHandler::paramCallback, this, _1, _2);
    m_configServer.setCallback(paramCallback);
  }
  
  ReconfigureHandler(std::string configName)
    : m_configServer(m_configMutex, ros::NodeHandle(configName))
  {
    auto paramCallback = boost::bind(&ReconfigureHandler::paramCallback, this, _1, _2);
    m_configServer.setCallback(paramCallback);
  }

  /**
   * @brief Return latest date from the reconfigure server
   *
   * @return const T&
   */
  const T &getData() { return m_currentConfig; }

private:
  void paramCallback(const T &cfgParams, uint32_t /* unused */)
  {
    m_currentConfig = std::move(cfgParams);
  }

  boost::recursive_mutex m_configMutex;
  dynamic_reconfigure::Server<T> m_configServer;
  T m_currentConfig;
};

}// namespace ros_util

#endif /* RECONFIGURE_HANDLER_HPP */