#ifndef BOID_HPP
#define BOID_HPP

#include "simconfig.hpp"
#include "vector2d.hpp"

#include <vector>

namespace bs {

class Boid
{
 private:
  Vector2D position_; //"real" class inveriant
  Vector2D velocity_; // soft constraint
  double max_vel_{35.0};
  double min_vel_{20.0};

 public:
  Boid(Vector2D const& pos, Vector2D const& vel)
      : position_{pos}
      , velocity_{vel}
  {}

  Vector2D const& get_position() const;

  Vector2D const& get_velocity() const;

  Vector2D compute_separation(std::vector<Boid> const& flock,
                              SimConfig const& config) const;

  Vector2D compute_alignment(std::vector<Boid> const& flock,
                             SimConfig const& config) const;

  Vector2D compute_cohesion(std::vector<Boid> const& flock,
                            SimConfig const& config) const;

  void limit_velocity();

  Boid update(std::vector<Boid> const& flock, SimConfig const& config) const;

  void apply_toroidal_boundary(SimConfig const& config);
  void apply_window_boundary(
      SimConfig const& config); // may modify velocity "soft constraint" (in
                                // update_physics is performed after .update)

  void
  apply_force(Vector2D const& force_point, double influence_radius,
              double interaction_strength); // may modify velocity "soft
                                            // constraint" (in update_physics is
                                            // performed after .update)

  void set_max_vel(double max_v);
  void set_min_vel(double min_v);
};

} // namespace bs
#endif