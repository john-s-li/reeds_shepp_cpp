#include "reeds_shepp.h"

namespace ReedsShepp {

  // utility functions
  float pi_2_pi(float angle) {
    return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
  }

  float mod2pi(float angle) {
    float v = fmod(angle, 2.0 * M_PI);
    
    if (v < -M_PI) {
      v += 2.0 * M_PI;
    }
    else if (v > M_PI) {
      v -= 2.0 * M_PI;
    }

    return v;
  }

  std::tuple<float, float> polar(float x, float y) {
    float r = sqrt(pow(x, 2) + pow(y, 2));
    float theta = atan2(y, x);
    return std::make_tuple(r, theta);
  }

  rs_tuple SLS(float x, float y, float phi) {
    phi = mod2pi(phi);

    float xd = -y / tan(phi) + x;
    float t  = xd - tan(phi / 2.0);
    float u  = phi;

    if (y > 0.0 && 0.0 < phi < M_PI * 0.99) {
      float v = sqrt(pow(x - xd, 2) + pow(y, 2)) - tan(phi / 2.0);
      return std::make_tuple(true, t, u, v);
    }
    else if (y < 0.0 < phi < M_PI * 0.99) {
      float v = -sqrt(pow(x - xd, 2) + pow(y, 2)) - tan(phi / 2.0);
      return std::make_tuple(true, t, u, v);
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
  }

  rs_tuple LSL(float x, float y, float phi) {
    auto [u, t] = polar(x - sin(phi), y - 1.0 + cos(phi));
    if (t >= 0.0) {
      float v = mod2pi(phi - t);
      if (v >= 0.0) {
        return std::make_tuple(true, t, u, v);
      }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
  }

  rs_tuple LRL(float x, float y, float phi) {
    auto [u1, t1] = polar(x - sin(phi), y - 1.0 + cos(phi));

    if (u1 <= 4.0) {
      float u = -2.0 * asin(u1 / 4.0);
      float t = mod2pi(t1 + 0.5 * u + M_PI);
      float v = mod2pi(phi - t + u);

      if (t >= 0.0 >= u) {
        return std::make_tuple(true, t, u, v);
      }

    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
  }

  void SCS(float x, float y, float phi, 
           std::vector<Path> paths, float step_size) {

    auto [flag, t, u, v] = SLS(x, y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'S', 'L', 'S'}, step_size);
    }

    auto [flag2, t2, u2, v2] = SLS(x, -y, -phi);

    if (flag2) {
      set_path(paths, std::vector<float>{t2, u2, v2},
               std::vector<char>{'S', 'R', 'S'}, step_size);
    }
  }

  void set_path(std::vector<Path> &paths, std::vector<float> lengths,  
                std::vector<char> ctypes, float step_size) {

    Path p; p.ctypes = ctypes; p.lengths = lengths;

    for (auto l: lengths) p.L += abs(l);

    // check whether this path exists already
    for (const auto path: paths) {
      bool same_types = (path.ctypes == p.ctypes);

      float path_l = 0.0;
      for (auto l: path.lengths) path_l += abs(l);

      bool close_lengths = abs(path_l - p.L) <= step_size;
      
      if (same_types && close_lengths) return;
    }

    // check whether path is long enough
    if (p.L <= step_size) return; // path too short, don't insert

    paths.push_back(p);
  }


}