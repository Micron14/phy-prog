#include "simulation.hpp"
#include "statistics.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace bs {

void update_physics(std::vector<Boid>& flock, std::vector<Boid>& kettle,
                    SimConfig const& config, SimConfig const& h_config,
                    bool left_mouse, bool right_mouse,
                    Vector2D const& mouse_pos, bool is_bounded)
{
  std::vector<Boid> next_flock;
  next_flock.reserve(flock.size());

  std::vector<Boid> next_kettle;
  next_kettle.reserve(kettle.size());

  Vector2D flock_center{get_flock_center(flock)};

  for (auto const& hunter : kettle) {
    Boid next_hunter = hunter.update(kettle, h_config);

    next_hunter.apply_force(hunter.get_position() + 4.0 * hunter.get_velocity(),
                            h_config.border_width,
                            h_config.interaction_strength); // "inertial force"
    next_hunter.apply_force(
        flock_center, h_config.border_width,
        -0.5 * h_config.interaction_strength); // "centripetal force"

    if (is_bounded) {
      next_hunter.apply_window_boundary(h_config);
    } else {
      next_hunter.apply_toroidal_boundary(h_config);
    }

    next_kettle.push_back(next_hunter);
  }

  for (auto const& boid : flock) {
    Boid next_boid = boid.update(flock, config);

    for (auto const& hunter : kettle) {
      next_boid.apply_force(hunter.get_position(), config.influence_radius,
                            config.interaction_strength);
    }

    if (left_mouse) {
      next_boid.apply_force(mouse_pos, config.influence_radius,
                            config.interaction_strength);
    } else if (right_mouse) {
      next_boid.apply_force(mouse_pos, config.influence_radius,
                            -config.interaction_strength);
    }

    if (is_bounded) {
      next_boid.apply_window_boundary(config);
    } else {
      next_boid.apply_toroidal_boundary(config);
    }

    next_flock.push_back(next_boid);
  }

  flock  = std::move(next_flock);
  kettle = std::move(next_kettle);
}

} // namespace bs