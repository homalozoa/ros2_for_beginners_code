#ifndef CHATTER_HPP
#define CHATTER_HPP

#include <string>

namespace ros_beginner
{
class Chatter
{
public:
  Chatter(const std::string & chatter_name);
  ~Chatter();
  std::string get_chatter_name() const;

private:
  std::string chatter_name;
};
}
#endif
