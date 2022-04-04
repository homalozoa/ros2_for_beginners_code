// Copyright (c) 2022 Homalozoa
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

#include <iostream>
#include <memory>

#include "pluginlib/class_loader.hpp"
#include "ch3_plugin_base/pluginbase.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<ch3::plugin::PluginBase> loader(
    "ch3_plugin_base",
    "ch3::plugin::PluginBase");
  // for (const auto & xml_path : loader.getPluginXmlPaths()) {
  //   std::cout << "xml path : " << xml_path << std::endl;
  // }
  // if (loader.isClassAvailable("ch3::plugin::PluginAlphaA")) {
  //   std::cout << "Ready to load 'ch3::plugin::PluginAlphaA'" << std::endl;
  //   loader.loadLibraryForClass("ch3::plugin::PluginAlphaA");
  // }
  // std::cout << "Load plugin "
  //   << loader.isClassAvailable("ch3::plugin::PluginAlphaA") << std::endl;

  try {
    auto alpha_a = loader.createUniqueInstance("ch3::plugin::PluginAlphaA");
    auto alpha_b = loader.createUniqueInstance("AlphaB");

    alpha_a->say_hello(2);
    alpha_b->say_hello(1);
  } catch (pluginlib::PluginlibException & ex) {
    printf("Failed to load PluginAlpha. Error: %s\n", ex.what());
  }

  try {
    auto beta_name = loader.createUniqueInstance("BetaPluginNewName");
    beta_name->say_something("using plugin name");
  } catch (pluginlib::PluginlibException & ex) {
    printf("Failed to load PluginBeta. Error: %s\n", ex.what());
  }

  try {
    auto gamma = loader.createUniqueInstance("ch3::plugin::PluginGamma");
    gamma->say_hello(1);
  } catch (pluginlib::PluginlibException & ex) {
    printf("Failed to load PluginGamma. Error: %s\n", ex.what());
  }

  return 0;
}
