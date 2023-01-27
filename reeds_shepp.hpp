#include <math.h>
#include <vector>
#include <tuple>

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

  // forward declarations
  void set_path(std::vector<Path> &paths, std::vector<float> lengths,  
                std::vector<char> ctypes, float step_size);

  //  ===================== utility functions ============================
  
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

  // ======================== motion primitives =========================

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
  } // end SLS

  rs_tuple LSL(float x, float y, float phi) {
    auto [u, t] = polar(x - sin(phi), y - 1.0 + cos(phi));
    if (t >= 0.0) {
      float v = mod2pi(phi - t);
      if (v >= 0.0) {
        return std::make_tuple(true, t, u, v);
      }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
  } // end LSL

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
  } // end LRL

  rs_tuple LSR(float x, float y, float phi) {
    auto [u1, t1] = polar(x + sin(phi), y - 1.0 - cos(phi));
    u1 = pow(u1, 2);
    if (u1 >= 4.0) {
      float u = sqrt(u1 - 4.0);
      float theta = atan2(2.0, u);
      float t = mod2pi(t1 + theta);
      float v = mod2pi(t - phi);

      if (t >= 0.0 && v >= 0.0) {
        return std::make_tuple(true, t, u, v);
      }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
  } // end LSR

  void SCS(float x, float y, float phi, 
           std::vector<Path> &paths, float step_size) {

    auto [flag, t, u, v] = SLS(x, y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'S', 'L', 'S'}, step_size);
    }

    std::tie(flag, t, u, v) = SLS(x, -y, -phi);

    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'S', 'R', 'S'}, step_size);
    }
  } // end SCS

  void CCC(float x, float y, float phi, 
           std::vector<Path> &paths, float step_size) {

    // forwards
    auto [flag, t, u, v] = LRL(x, y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'L', 'R', 'L'}, step_size);
    }

    std::tie(flag, t, u, v) = LRL(-x, y, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{-t, -u, -v},
               std::vector<char>{'L', 'R', 'L'}, step_size);
    }

     std::tie(flag, t, u, v) = LRL(x, -y, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'R', 'L', 'R'}, step_size);
    }

    std::tie(flag, t, u, v) = LRL(-x, -y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{-t, -u, -v},
               std::vector<char>{'R', 'L', 'R'}, step_size);
    }

    // backwards
    float xb = x * cos(phi) + y * sin(phi);
    float yb = x * cos(phi) - y * sin(phi);

    std::tie(flag, t, u, v) = LRL(xb, yb, phi);
    if (flag) {
      set_path(paths, std::vector<float>{v, u, t},
               std::vector<char>{'L', 'R', 'L'}, step_size);
    }

    std::tie(flag, t, u, v) = LRL(-xb, yb, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{-v, -u, -t},
               std::vector<char>{'L', 'R', 'L'}, step_size);
    }

    std::tie(flag, t, u, v) = LRL(xb, -yb, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{v, u, t},
               std::vector<char>{'R', 'L', 'R'}, step_size);
    }
    
    std::tie(flag, t, u, v) = LRL(-xb, -yb, phi);
    if (flag) {
      set_path(paths, std::vector<float>{-v, -u, -t},
               std::vector<char>{'R', 'L', 'R'}, step_size);
    }

  } // end CCC

  void CSC(float x, float y, float phi, 
           std::vector<Path> &paths, float step_size) {

    auto [flag, t, u, v] = LSL(x, y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'L', 'S', 'L'}, step_size);
    }

    std::tie(flag, t, u, v) = LSL(-x, y, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{-t, -u, -v},
               std::vector<char>{'L', 'S', 'L'}, step_size);
    }

    std::tie(flag, t, u, v) = LSL(x, -y, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'R', 'S', 'R'}, step_size);
    }

    std::tie(flag, t, u, v) = LSL(-x, -y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{-t, -u, -v},
               std::vector<char>{'R', 'S', 'R'}, step_size);
    }

    std::tie(flag, t, u, v) = LSR(x, y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'L', 'S', 'R'}, step_size);
    }

    std::tie(flag, t, u, v) = LSR(-x, y, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{-t, -u, -v},
               std::vector<char>{'L', 'S', 'R'}, step_size);
    }

    std::tie(flag, t, u, v) = LSR(x, -y, -phi);
    if (flag) {
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'R', 'S', 'L'}, step_size);
    }

    std::tie(flag, t, u, v) = LSR(-x, -y, phi);
    if (flag) {
      set_path(paths, std::vector<float>{-t, -u, -v},
               std::vector<char>{'R', 'S', 'L'}, step_size);
    }

  } // end CSC

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