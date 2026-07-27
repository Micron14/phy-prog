#ifndef STATISTICS_HPP
#define STATISTICS_HPP

#include "boid.hpp"
#include "vector2d.hpp"

#include <vector>

namespace stats {

struct StatResult
{
  double mean_;
  double std_dev_;
};

StatResult avg_speed(std::vector<boids::Boid> const& flock);

StatResult avg_position(std::vector<boids::Boid> const& flock);

std::vector<int>
calculate_speed_histogram(std::vector<boids::Boid> const& flock,
                          size_t num_bins, double max_val);

phy::Vector2D get_flock_center(std::vector<boids::Boid> const& flock);

} // namespace stats
#endif
