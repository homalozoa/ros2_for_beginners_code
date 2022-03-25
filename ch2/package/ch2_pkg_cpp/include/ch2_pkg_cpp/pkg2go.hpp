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

#ifndef CH2_PKG_CPP__PKG2GO_HPP_
#define CH2_PKG_CPP__PKG2GO_HPP_

#include <string>

namespace ros_beginner
{
class Pkg2Go
{
public:
  explicit Pkg2Go(const std::string & pkg2go_name);
  ~Pkg2Go();
  std::string get_pkg2go_name() const;

private:
  std::string pkg2go_name_;
};
}  // namespace ros_beginner
#endif  // CH2_PKG_CPP__PKG2GO_HPP_
