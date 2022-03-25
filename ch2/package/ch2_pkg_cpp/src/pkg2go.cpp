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

#include <string>

#include "ch2_pkg_cpp/pkg2go.hpp"

namespace ros_beginner
{
Pkg2Go::Pkg2Go(const std::string & pkg2go_name)
{
  this->pkg2go_name_ = pkg2go_name;
}

Pkg2Go::~Pkg2Go()
{}

std::string Pkg2Go::get_pkg2go_name() const
{
  return this->pkg2go_name_;
}
}  // namespace ros_beginner
