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
  std::vector<Boid> next_flock{flock};
  std::vector<Boid> next_kettle{kettle};
  Vector2D flock_center{get_flock_center(flock)};

  for (auto& hunter : next_kettle) {
    hunter.apply_force(hunter.get_position() + 4.0 * hunter.get_velocity(),
                       h_config.border_width,
                       h_config.interaction_strength); // "inertial force"
    hunter.apply_force(
        flock_center, h_config.border_width,
        -0.5 * h_config.interaction_strength); // "centripetal force"
    if (is_bounded) {
      hunter.apply_window_boundary(h_config);
    } else {
      hunter.apply_toroidal_boundary(h_config);
    }
    hunter.update(kettle, h_config);
  }

  for (auto& boid : next_flock) {
    boid.update(flock, config);

    for (auto const& hunter : kettle) {
      boid.apply_force(hunter.get_position(), config.influence_radius,
                       config.interaction_strength);
    }

    if (left_mouse) {
      boid.apply_force(mouse_pos, config.influence_radius,
                       config.interaction_strength);
    } else if (right_mouse) {
      boid.apply_force(mouse_pos, config.influence_radius,
                       -config.interaction_strength);
    }

    if (is_bounded) {
      boid.apply_window_boundary(config);
    } else {
      boid.apply_toroidal_boundary(config);
    }
  }
  flock  = std::move(next_flock);
  kettle = std::move(next_kettle);
}

} // namespace bs