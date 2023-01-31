#include "reeds_shepp.hpp"

int main() {
  auto test_list = 
    ReedsShepp::arange<float>(0.0, 1.0, 0.25);

  for (auto t: test_list) {
    std::cout << t << " ";
  }

  return 0;
}