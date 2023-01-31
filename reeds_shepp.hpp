#include <math.h>
#include <vector>
#include <tuple>
#include <array>
#include <iostream>

namespace ReedsShepp {

  typedef std::tuple<bool, float, float, float> rs_tuple;
  typedef std::vector<std::vector<float>> interp_dist_list;
  typedef float pose[3];

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

  template <typename T>
  std::vector<T> arange(T start, T stop, T step) {
    /*
     * Utility function to emulate behavior similar to that of numpy arange()
     */

    std::vector<T> l(static_cast<int>(stop / step));
    T curr = l[0] = start;
    std::generate(l.begin() + 1, l.end(),
                  [&]() -> T { curr += step; return curr;
                  });

    return l;
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
    } // end CCC

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

  // ====================== Core Functions ==============================

  void set_path(std::vector<Path> &paths, std::vector<float> lengths,  
                std::vector<char> ctypes, float step_size) {

    Path p; p.ctypes = ctypes; p.lengths = lengths;

    for (auto l: lengths) p.L += abs(l);

    // check whether path is long enough
    if (p.L <= step_size) return; // path too short, don't insert

    // check whether this path exists already
    for (const auto path_i: paths) {
      bool same_types = (path_i.ctypes == p.ctypes);

      float l_path_i = 0.0;
      for (auto l: path_i.lengths) l_path_i += abs(l);

      bool close_lengths = abs(l_path_i - p.L) <= step_size;
      
      if (same_types && close_lengths) return;
    }
  
    paths.push_back(p);
  } // end set_path

  std::vector<Path> generate_path(pose q0, pose q1, 
                                  float max_curvatuve, float step_size) {
    float dx = q1[0] - q0[0];
    float dy = q1[1] - q0[1];
    float dth = q1[2] - q0[2];
    
    float c = cos(q0[2]);
    float s = sin(q0[2]);

    float x = (c * dx + s * dy) * max_curvatuve;
    float y = (-s * dx + c * dy) * max_curvatuve;

    std::vector<Path> paths;
    SCS(x, y, dth, paths, step_size);
    CSC(x, y, dth, paths, step_size);
    CCC(x, y, dth, paths, step_size);

    return paths;
  } // end generate_path

  interp_dist_list calc_interpolate_dists_list(std::vector<float> lengths,
                                               float step_size) {
    interp_dist_list idl;
    for (auto l: lengths) {
      float d_dist = l >= 0.0 ? step_size : -step_size;
      auto interp_dists = arange<float>(0.0, l, d_dist);
      interp_dists.push_back(l);
      idl.push_back(interp_dists);
    }

    return idl;
  } // end calc_interpolate_dists_list

}