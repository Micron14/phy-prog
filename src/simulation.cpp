#include "simulation.hpp"
#include "statistics.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace sim {
void update_physics(std::vector<boids::Boid>& flock,
                    std::vector<boids::Boid>& kettle,
                    boids::SimConfig const& config,
                    boids::SimConfig const& h_config, bool left_mouse,
                    bool right_mouse, phy::Vector2D const& mouse_pos,
                    bool is_bounded)
{
  std::vector<boids::Boid> next_flock{flock};
  std::vector<boids::Boid> next_kettle{kettle};
  phy::Vector2D flock_center{stats::get_flock_center(flock)};

  for (std::size_t i{0}; i < next_kettle.size(); ++i) {
    next_kettle[i].apply_force(
        next_kettle[i].get_position() + 4.0 * next_kettle[i].get_velocity(),
        h_config.border_width,
        h_config.interaction_strength); // "inertial force"
    next_kettle[i].apply_force(
        flock_center, h_config.border_width,
        -0.5 * h_config.interaction_strength); // "centripetal force"
    if (is_bounded) {
      next_kettle[i].apply_window_boundary(h_config);
    } else {
      next_kettle[i].apply_toroidal_boundary(h_config);
    }
    next_kettle[i].update(kettle, i, h_config);
  }

  for (std::size_t i{0}; i < next_flock.size(); ++i) {
    next_flock[i].update(flock, i, config);

    for (const auto& hunter : kettle) {
      next_flock[i].apply_force(hunter.get_position(), config.influence_radius,
                                config.interaction_strength);
    }

    if (left_mouse) {
      next_flock[i].apply_force(mouse_pos, config.influence_radius,
                                config.interaction_strength);
    } else if (right_mouse) {
      next_flock[i].apply_force(mouse_pos, config.influence_radius,
                                -config.interaction_strength);
    }

    if (is_bounded) {
      next_flock[i].apply_window_boundary(config);
    } else {
      next_flock[i].apply_toroidal_boundary(config);
    }
  }
  flock  = std::move(next_flock);
  kettle = std::move(next_kettle);
}
} // namespace sim