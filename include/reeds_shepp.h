#include <math.h>
#include <vector>
#include <tuple>

#include "NumCpp.hpp"

namespace ReedsShepp {

  typedef std::tuple<bool, float, float, float> rs_tuple;

  // helper data container
  struct Path {
    std::vector<float> lengths;  // segment lengths, negative = backwards
    std::vector<char>  ctypes;   // S: straight, L: left, R: right
    std::vector<float> x;        // x-positions
    std::vector<float> y;        // y-positions
    std::vector<float> yaw;      // orientations [rad]
    std::vector<int> directions; // 1: forward, -1: backwards
    float L{0.0};                // total length of path 
  }; 

  void set_path(std::vector<Path> &paths, std::vector<float> lengths,  
                std::vector<char> ctypes, float step_size = 0.1);

  rs_tuple SLS(float x, float y, float phi);
  rs_tuple LSL(float x, float y, float phi);
  rs_tuple LRL(float x, float y, float phi);
  rs_tuple CCC(float x, float y, float phi);
  rs_tuple CSC(float x, float y, float phi);
  rs_tuple LSR(float x, float y, float phi);

  void SCS(float x, float y, float phi, 
           std::vector<Path> paths, float step_size);

} // end namespace ReedsShepp