#include <math.h>
#include <vector>
#include <tuple>
#include <array>
#include <iostream>

namespace ReedsShepp {

  typedef std::tuple<bool, float, float, float> rs_tuple;
  typedef std::tuple<float, float, float, int> interp_tuple;
  typedef std::tuple<std::vector<float>, std::vector<float>,
                     std::vector<float>, std::vector<int>> lc_return;
  
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

    Path& operator=(const Path& other) {
      lengths = other.lengths;
      ctypes  = other.ctypes;
      x = other.x;
      y = other.y;
      yaw = other.yaw;
      L = other.L;

      return *this;
    }

    friend std::ostream& operator<<(std::ostream& os,const Path& path);
  }; 

  std::ostream& operator<<(std::ostream& os,const Path& path) {
      // prints out the lenghts of each segment in the path
      os << "[ ";
      for (int i = 0; i < path.lengths.size() - 1; i++) {
        os << path.lengths[i] << " , ";
      }
      os << path.lengths.back() << " ]\n";

      return os;
    }

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
     * returns a vector of size stop/step with elements [start, stop]
     */

    std::vector<T> l(static_cast<int>(stop / step) + 1);
    T curr = l[0] = start;
    std::generate(l.begin() + 1, l.end(),
                  [&]() -> T { curr += step; return curr;
                  });

    return l;
  }

  template <typename T>
  T deg2rad(T degree) {
    return (degree * M_PI) / 180.0;
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
    else if ((y < 0.0 && y < phi && y < M_PI * 0.99) &&
             (phi > 0.0 && phi < 0.99 * M_PI)) {
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

      if (t >= 0.0 && 0.0 >= u) {
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
      std::cout << "SLS true" << std::endl;
      set_path(paths, std::vector<float>{t, u, v},
               std::vector<char>{'S', 'L', 'S'}, step_size);
    }

    std::tie(flag, t, u, v) = SLS(x, -y, -phi);

    if (flag) {
      std::cout << "SLS neg true" << std::endl;

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
    float yb = x * sin(phi) - y * cos(phi);

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

    Path p; 
    p.ctypes = ctypes; p.lengths = lengths;

    for (auto l: lengths) p.L += abs(l);

    // check whether path is long enough
    if (p.L <= step_size) {
      return; // path too short, don't insert
    }

    // check whether this path exists already
    for (const auto path_i: paths) {
      bool same_types = (path_i.ctypes == p.ctypes);

      float l_path_i = 0.0;
      for (auto l: path_i.lengths) l_path_i += abs(l);

      bool close_lengths = abs(l_path_i - p.L) <= step_size;
      
      if (same_types && close_lengths) {
        return;
      }
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
    std::cout << "SCS path len = " << paths.size() << std::endl;
    CSC(x, y, dth, paths, step_size);
    std::cout << "CSC path len = " << paths.size() << std::endl;
    CCC(x, y, dth, paths, step_size);
    std::cout << "CCC path len = " << paths.size() << std::endl;

    return paths;
  } // end generate_path

  interp_dist_list calc_interpolate_dists_list(std::vector<float> lengths,
                                               float step_size) {
    interp_dist_list idl;
    for (auto l: lengths) {
      float d_dist = l >= 0.0 ? step_size : -step_size;
      std::vector<float> interp_dists = arange<float>(0.0, l, d_dist);
      interp_dists.push_back(l);
      idl.push_back(interp_dists);
    }

    return idl;
  } // end calc_interpolate_dists_list

  interp_tuple interpolate(float dist, float length,
                           char mode, float max_curvature, 
                           float ox, float oy, float oyaw) {

    float x, y, yaw; 
    int direction = length > 0.0 ? 1 : -1;

    if (mode == 'S') { // straight
      x = ox + dist / max_curvature * cos(oyaw);
      y = oy + dist / max_curvature * sin(oyaw);
      yaw = oyaw;
    }
    else { // curve
      float ldx = sin(dist) / max_curvature;
      float ldy = 0.0;
      
      switch (mode) {
        case 'L':
          ldy = (1.0 - cos(dist)) / max_curvature;
          yaw = oyaw + dist;
          break;
        case 'R':
          ldy = (1.0 - cos(dist)) / -max_curvature;
          yaw = oyaw - dist;
          break;
      }

      float gdx = cos(-oyaw) * ldx + sin(-oyaw) * ldy;
      float gdy = -sin(-oyaw) * ldx + cos(-oyaw) * ldy;

      x = ox + gdx;
      y = oy + gdy;
    }

    return std::make_tuple(x, y, yaw, direction);
  } // end interpolate

  lc_return generate_local_course(std::vector<float> lengths,
                                  std::vector<char>  modes,
                                  float max_curvature, float step_size) {

    float x, y, yaw; int direction;
    
    interp_dist_list idl = calc_interpolate_dists_list(lengths, step_size);
    
    float origin_x , origin_y, origin_yaw = 0.0;

    std::vector<float> xs, ys, yaws;
    std::vector<int> directions;
  
    int min_len = (lengths.size() == idl.size() == modes.size()) ?
                   lengths.size() : 
                   std::min({lengths.size(), idl.size(), modes.size()});

    for (int i = 0; i < min_len; i++) {
      auto interp_dists = idl[i];
      auto mode = modes[i];
      auto length = lengths[i];

      for (auto dist: interp_dists) {
        std::tie(x, y, yaw, direction) = interpolate(dist, length, mode,
                                                     max_curvature, origin_x,
                                                     origin_y, origin_yaw);

        xs.push_back(x);
        ys.push_back(y);
        yaws.push_back(yaw);
        directions.push_back(direction);
      }

      origin_x = xs.back();
      origin_y = ys.back();
      origin_yaw = yaws.back();
    }

    return std::make_tuple(xs, ys, yaws, directions);
  } // end generate_local_course

  std::vector<Path> calc_paths(float sx, float sy, float syaw,
                               float gx, float gy, float gyaw,
                               float maxc, float step_size) {

    pose q0 = {sx, sy, syaw};
    pose q1 = {gx, gy, gyaw};

    auto paths = generate_path(q0, q1, maxc, step_size);

    std::vector<float> xs, ys, yaws; 
    std::vector<int> directions;

    for (auto& path_i: paths) {
      std::tie(xs, ys, yaws, directions) = generate_local_course(
                                                    path_i.lengths,
                                                    path_i.ctypes,
                                                    maxc, step_size * maxc);
      
      std::vector<float> x_local, y_local, yaw_local;

      // convert path to global coordinates
      // length of xs, ys, and yaws are the same
      for (int i = 0; i < xs.size(); i++) {
        x_local.push_back(cos(-q0[2]) * xs[i] + sin(-q0[2]) * ys[i] + q0[0]);
        y_local.push_back(-sin(-q0[2]) * xs[i] + cos(-q0[2]) * ys[i] + q0[1]);
        yaw_local.push_back((yaws[i] + q0[2]));
      }

      path_i.x = x_local;
      path_i.y = y_local;
      path_i.yaw = yaw_local;
      path_i.directions = directions;

      for (int i = 0; i < path_i.lengths.size(); i++) {
        //printf("path_i elem before = %.4f\n", path_i.lengths[i]);
        path_i.lengths[i] /= maxc;
        //printf("path_i elem after = %.4f\n", path_i.lengths[i]);
      }

      path_i.L /= maxc;
    }

    return paths;
  } // end calc_paths

  Path* reeds_shepp_path_planning(float sx, float sy, float syaw,
                                  float gx, float gy, float gyaw,
                                  float maxc, float step_size = 0.2) {

    static std::vector<Path> paths; // to put local path vector element on 
                                    // stack into the heap instead for external
                                    // access
    paths.clear();

    paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size);

    if (paths.empty()) return nullptr;

    // search for the minimum cost path
    sort(paths.begin(), paths.end(),
         [](const Path& a, const Path& b) -> bool {
           return abs(a.L) < abs(b.L); 
         }
    );

    return &(paths[0]);
  } // end reeds_shepp_path_planning

}