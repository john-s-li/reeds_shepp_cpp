#include <math.h>
#include <vector>

namespace ReedsShepp {

  struct Path {
    std::vector<float> lengths;  // segment lengths, negative = backwards
    std::vector<char>  types;    // S: straight, L: left, R: right
    std::vector<float> x;        // x-positions
    std::vector<float> y;        // y-positions
    std::vector<float> yaw;      // orientations [rad]
    std::vector<int> directions; // 1: forward, -1: backwards
    float L{0.0};                // total length of path 
  }; 

} // end namespace ReedsShepp