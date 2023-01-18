cmake_minimum_required(VERSION 3.0)

project(reeds-shepp)

add_executable(test_reeds_shepp test.cpp)
set_target_properties(test_reeds_shepp PROPERTIES CXX_STANDARD 17)