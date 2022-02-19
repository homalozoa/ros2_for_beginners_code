// Copyright 2022 Homalozoa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CH3_PLUGIN_BASE__PLUGINBASE_HPP_
#define CH3_PLUGIN_BASE__PLUGINBASE_HPP_

#include <string>

namespace ch3
{
namespace plugin
{
class PluginBase
{
public:
  ~PluginBase() {}
  virtual void say_hello(const int32_t & times) = 0;
  virtual bool say_something(const std::string & something) = 0;

protected:
  PluginBase() = default;
};
}  // namespace plugin
}  // namespace ch3
#endif  // CH3_PLUGIN_BASE__PLUGINBASE_HPP_
