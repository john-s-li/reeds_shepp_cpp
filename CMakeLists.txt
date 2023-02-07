cmake_minimum_required(VERSION 3.0)

project(reeds-shepp)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Python3 3.0 REQUIRED)

include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})


add_executable(plot_reeds_shepp plot_path.cpp)

target_include_directories(
    plot_reeds_shepp
    PRIVATE
        ${Python3_INCLUDE_DIRS}
        ${NUMPY_INCLUDE_DIRS}
)

target_link_libraries(
    plot_reeds_shepp
    ${Python3_LIBRARIES}
    Python3::NumPy
)

set_target_properties(plot_reeds_shepp PROPERTIES CXX_STANDARD 17)