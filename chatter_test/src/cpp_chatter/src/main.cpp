#include <iostream>

#include <cpp_chatter/chatter.hpp>

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  auto chatter_str = ros_beginner::Chatter("Hello ROS 2.");
  std::cout << chatter_str.get_chatter_name() << std::endl;
  return 0;
}
