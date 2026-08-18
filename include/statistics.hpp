#ifndef STATISTICS_HPP
#define STATISTICS_HPP

#include "boid.hpp"
#include "vector2d.hpp"

#include <vector>

namespace bs {

struct StatResult
{
  double mean;
  double std_dev;
};

StatResult avg_speed(std::vector<Boid> const& flock);

StatResult avg_position(std::vector<Boid> const& flock);

std::vector<int> calculate_speed_histogram(std::vector<Boid> const& flock,
                                           size_t num_bins, double max_val);

Vector2D get_flock_center(std::vector<Boid> const& flock);

void print_stats(std::vector<Boid> const& flock);

} // namespace bs
#endif
