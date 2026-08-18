#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "simconfig.hpp"
#include "vector2d.hpp"

#include <vector>

namespace bs {

void update_physics(std::vector<Boid>& flock, std::vector<Boid>& kettle,
                    SimConfig const& config, SimConfig const& h_config,
                    bool left_mouse, bool right_mouse,
                    Vector2D const& mouse_pos, bool is_bounded);

std::vector<Boid> entity_gen(SimConfig const& config);

} // namespace bs
#endif