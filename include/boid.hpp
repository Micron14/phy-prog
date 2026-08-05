#ifndef BOID_HPP
#define BOID_HPP

#include "simconfig.hpp"
#include "vector2d.hpp"

#include <cstddef>
#include <vector>

namespace boids {

class Boid
{
 private:
  Vector2D position_;
  Vector2D velocity_;
  double max_vel_{35.0};
  double min_vel_{20.0};

 public:
  Boid(Vector2D pos, Vector2D vel)
      : position_{pos}
      , velocity_{vel}
  {}

  Vector2D get_position() const;

  Vector2D get_velocity() const;

  Vector2D compute_separation(const std::vector<Boid>& flock,
                                   std::size_t self_index,
                                   SimConfig const& config) const;

  Vector2D compute_alignment(const std::vector<Boid>& flock,
                                  std::size_t self_index,
                                  SimConfig const& config) const;

  Vector2D compute_cohesion(const std::vector<Boid>& flock,
                                 std::size_t self_index,
                                 SimConfig const& config) const;

  void limit_velocity();

  void update(const std::vector<Boid>& flock, std::size_t self_index,
              SimConfig const& config);

  void apply_toroidal_boundary(SimConfig const& config);
  void apply_window_boundary(SimConfig const& config);

  void apply_force(const Vector2D& force_point, double influence_radius,
                   double interaction_strength);

  void set_max_vel(double max_v);
  void set_min_vel(double min_v);
};

std::vector<Boid> entity_gen(SimConfig const& config);

} // namespace boids
#endif