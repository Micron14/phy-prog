#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "simconfig.hpp"
#include "vector2d.hpp"

#include <vector>

namespace sim {
void update_physics(std::vector<boids::Boid>& flock,
                    std::vector<boids::Boid>& kettle,
                    boids::SimConfig const& config,
                    boids::SimConfig const& h_config, bool left_mouse,
                    bool right_mouse, phy::Vector2D const& mouse_pos,
                    bool is_bounded);
}

#endif