cmake_minimum_required(VERSION 3.8)
project(test_mpg123)

find_package(mpg123_vendor REQUIRED)

message(${mpg123_vendor_INCLUDE_DIRS})