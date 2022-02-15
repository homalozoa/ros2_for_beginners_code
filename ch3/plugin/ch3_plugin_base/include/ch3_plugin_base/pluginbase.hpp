#ifndef CH3_PLUGIN_BASE__PLUGINBASE__HPP_
#define CH3_PLUGIN_BASE__PLUGINBASE__HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace plugintest
{
class RealPluginBase
{
public:
  ~RealPluginBase(){}
  virtual say_hello(const uint32_t & times) = 0;
  virtual say_something(const std::string & something) = 0;
protected:
  RealPluginBase() = default;
};

template<T SomeT>
class FakePluginBase
{
public:
  ~FakePluginBase(){}
  virtual say_hello(const uint32_t & times) = 0;
  virtual say_something(const SomeT & something) = 0;
protected:
  RealPluginBase() = default;
};

}  // namespace plugintest
#endif  // CH3_PLUGIN_BASE__PLUGINBASE__HPP_
