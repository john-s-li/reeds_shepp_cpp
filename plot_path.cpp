#include <iostream>
#include <string>

#include "matplotlibcpp.h"
#include "reeds_shepp.hpp"

namespace plt = matplotlibcpp;

void draw_arrow(float x, float y, float theta, 
                float length = 1.0, float line_width = 3.0, 
                std::string color = "k") {
      
  float angle = 30.0 / 180.0 * M_PI; // angle of the arrow head
  float d = 0.3 * length;

  std::map<std::string, std::string> settings;
  settings["linewidth"] = std::to_string(line_width);
  settings["color"] = color;

  // need to ensure that theta is in radians (simple check)
  if (theta > 2*M_PI) {
    theta = theta / 180.0 * M_PI;
  } 

  float x_start = x;
  float y_start = y;
  float x_end = x + length * cos(theta);
  float y_end = y + length * sin(theta);  

  float theta_hat_L = theta + M_PI - angle;
  float theta_hat_R = theta + M_PI + angle;

  float x_hat_start = x_end;
  float x_hat_end_L = x_hat_start + d * cos(theta_hat_L);
  float x_hat_end_R = x_hat_start + d * cos(theta_hat_R);

  float y_hat_start = y_end;
  float y_hat_end_L = y_hat_start + d * sin(theta_hat_L);
  float y_hat_end_R = y_hat_start + d * sin(theta_hat_R);

  plt::plot(std::vector<float>{x_start, x_end},
            std::vector<float>{y_start, y_end}, settings);

  plt::plot(std::vector<float>{x_hat_start, x_hat_end_L},
            std::vector<float>{y_hat_start, y_hat_end_L}, settings);

  plt::plot(std::vector<float>{x_hat_start, x_hat_end_R},
            std::vector<float>{y_hat_start, y_hat_end_R}, settings);
}


int main() {
  std::cout << "Running Reeds-Shepp Path Planning" << std::endl;

  const bool SHOW_ANIMATION = true;

  float start_x = 1.0;
  float start_y = 2.0;
  float start_yaw = ReedsShepp::deg2rad<float>(30.0);

  float end_x = 5.0;
  float end_y = 3.0;
  float end_yaw = ReedsShepp::deg2rad<float>(25.0);

  float curvature = 0.1;
  float step_size = 0.05;

  ReedsShepp::Path* best_path = ReedsShepp::reeds_shepp_path_planning(
                                                  start_x, start_y, start_yaw,
                                                  end_x, end_y, end_yaw,
                                                  curvature, step_size);

  std::cout << "Best path lenghts = " << *best_path;  

  std::string motion_name(best_path->ctypes.begin(),
                          best_path->ctypes.end());

  if (SHOW_ANIMATION) {
    
    for (int i = 0; i < best_path->x.size(); i++) {
      plt::clf();
      draw_arrow(start_x, start_y, start_yaw);
      draw_arrow(end_x, end_y, end_yaw);
      plt::plot(best_path->x, best_path->y);
      draw_arrow(best_path->x[i], best_path->y[i], best_path->yaw[i]);
      plt::title("Reeds-Shepp Paths with Motion Primitive " + motion_name);
      plt::grid(true);
      plt::axis("equal");
      plt::pause(0.01);
    }

  }
   
  std::cout << "Press enter to close ..." << std::endl;
  std::cin.ignore();

  return 0;
}