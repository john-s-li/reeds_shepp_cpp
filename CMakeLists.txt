cmake_minimum_required(VERSION 3.0)

project(reeds-shepp)

#find_package(Boost 1.80 COMPONENTS program_options REQUIRED)
#include_directories(include)

add_executable(test_reeds_shepp test.cpp)

# target_include_directories(test_reeds_shepp
#     PRIVATE
#         ${Boost_INCLUDE_DIRS})

set_target_properties(test_reeds_shepp PROPERTIES CXX_STANDARD 17)