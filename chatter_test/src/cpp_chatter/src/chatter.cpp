#include <string>

#include <cpp_chatter/chatter.hpp>

namespace ros_beginner
{
Chatter::Chatter(const std::string & chatter_name)
{
  this->chatter_name = chatter_name;
}

Chatter::~Chatter()
{}

std::string Chatter::get_chatter_name() const
{
  return this->chatter_name;
}
}
