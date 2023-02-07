#include <iostream>

#include "reeds_shepp.hpp"

int main() {
  std::cout << "Running Reeds-Shepp Path Planning" << std::endl;

  float start_x = -2.0;
  float start_y = -5.0;
  float start_yaw = ReedsShepp::deg2rad<float>(-30.0);

  float end_x = 5.0;
  float end_y = 5.0;
  float end_yaw = ReedsShepp::deg2rad<float>(25.0);

  float curvature = 0.1;
  float step_size = 0.05;

  ReedsShepp::Path* best_path = ReedsShepp::reeds_shepp_path_planning(
                                                  start_x, start_y, start_yaw,
                                                  end_x, end_y, end_yaw,
                                                  curvature, step_size);

  std::cout << "Best path lenghts = " << *best_path;     

  return 0;
}