#ifndef STATISTICS_HPP
#define STATISTICS_HPP

#include "boid.hpp"
#include "vector2d.hpp"

#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace bs {

struct StatResult
{
  double mean;
  double std_dev;
};

template<typename UnaryFunction>
StatResult avg_measure(std::vector<Boid> const& flock, UnaryFunction func)
{
  if (flock.empty()) {
    return {0.0, 0.0};
  }
  double const N{static_cast<double>(flock.size())};
  double mean_val{std::transform_reduce(flock.begin(), flock.end(), 0.0,
                                        std::plus<>(), func)
                  / N};

  double variance{
      std::transform_reduce(flock.begin(), flock.end(), 0.0, std::plus<>(),
                            [mean_val, &func](Boid const& b) {
                              double v{func(b)};
                              return (v - mean_val) * (v - mean_val);
                            })
      / N};
  return {mean_val, std::sqrt(variance)};
}

StatResult avg_speed(std::vector<Boid> const& flock);

StatResult avg_position(std::vector<Boid> const& flock);

std::vector<int> calculate_speed_histogram(std::vector<Boid> const& flock,
                                           std::size_t num_bins,
                                           double max_val);

Vector2D get_flock_center(std::vector<Boid> const& flock);

void print_stats(std::vector<Boid> const& flock);

} // namespace bs
#endif
