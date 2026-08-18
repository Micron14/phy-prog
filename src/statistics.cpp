#include "statistics.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace bs {

StatResult avg_speed(std::vector<Boid> const& flock)
{
  if (flock.empty()) {
    return {0.0, 0.0};
  }
  double const N{static_cast<double>(flock.size())};
  double mean_speed{std::transform_reduce(
                        flock.begin(), flock.end(), 0.0, std::plus<>(),
                        [](Boid const& b) { return b.get_velocity().norm(); })
                    / N};

  double variance{
      std::transform_reduce(flock.begin(), flock.end(), 0.0, std::plus<>(),
                            [mean_speed](Boid const& b) {
                              double v{b.get_velocity().norm()};
                              return (v - mean_speed) * (v - mean_speed);
                            })
      / N};
  return {mean_speed, std::sqrt(variance)};
}

StatResult avg_position(std::vector<Boid> const& flock)
{
  if (flock.empty()) {
    return {0.0, 0.0};
  }
  double const N{static_cast<double>(flock.size())};
  double mean_position{std::transform_reduce(flock.begin(), flock.end(), 0.0,
                                             std::plus<>(),
                                             [](Boid const& b) {
                                               return b.get_position().norm();
                                             })
                       / N};

  double variance{
      std::transform_reduce(flock.begin(), flock.end(), 0.0, std::plus<>(),
                            [mean_position](Boid const& b) {
                              double p{b.get_position().norm()};
                              return (p - mean_position) * (p - mean_position);
                            })
      / N};
  return {mean_position, std::sqrt(variance)};
}

std::vector<int> calculate_speed_histogram(std::vector<Boid> const& flock,
                                           size_t num_bins, double max_val)
{
  if (num_bins == 0) {
    throw std::runtime_error("Histogram must have at least 1 bin.");
  }
  if (max_val <= 0.0) {
    throw std::runtime_error(
        "Maximum value for histogram must be strictly positive.");
  }

  std::vector<int> bins(num_bins, 0);

  for (auto const& b : flock) {
    double val{b.get_velocity().norm()};
    size_t bin_idx{
        static_cast<size_t>((val / max_val) * static_cast<double>(num_bins))};

    if (bin_idx >= num_bins) {
      bin_idx = num_bins - 1;
    }

    ++bins[bin_idx];
  }
  return bins;
}

Vector2D get_flock_center(std::vector<Boid> const& flock)
{
  if (flock.empty()) {
    return {0.0, 0.0};
  }
  double const N{static_cast<double>(flock.size())};
  Vector2D flock_center{
      std::transform_reduce(flock.begin(), flock.end(), Vector2D{0.0, 0.0},
                            std::plus<>(),
                            [](Boid const& b) { return b.get_position(); })
      / N};

  return flock_center;
}

void print_stats(std::vector<Boid> const& flock)
{
  StatResult speed{avg_speed(flock)};
  StatResult position{avg_position(flock)};
  std::cout << "Mean speed: " << speed.mean << " | Std. Dev.: " << speed.std_dev
            << " | Mean position: " << position.mean
            << " | Std. Dev.: " << position.std_dev << '\n';
}

} // namespace bs
