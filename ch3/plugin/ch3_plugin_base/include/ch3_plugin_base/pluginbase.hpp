#ifndef CH3_PLUGIN_BASE__PLUGINBASE__HPP_
#define CH3_PLUGIN_BASE__PLUGINBASE__HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace pluginvirtual
{
class RealPluginBase
{
public:
  ~RealPluginBase(){}
  void virtual say_hello(const uint32_t & times) = 0;
  bool virtual say_something(const std::string & something) = 0;
protected:
  RealPluginBase() = default;
};

template<T SomeT>
class FakePluginBase
{
public:
  ~FakePluginBase(){}
  void virtual say_hello(const uint32_t & times) = 0;
  bool virtual say_something(const SomeT & something) = 0;
protected:
  RealPluginBase() = default;
};

}  // namespace pluginvirtual
#endif  // CH3_PLUGIN_BASE__PLUGINBASE__HPP_
