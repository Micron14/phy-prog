#include "boid.hpp"
#include "simconfig.hpp"
#include "vector2d.hpp"

#include <random>
#include <vector>

namespace boids {
phy::Vector2D Boid::get_position() const
{
  return position_;
}

phy::Vector2D Boid::get_velocity() const
{
  return velocity_;
}

phy::Vector2D Boid::compute_separation(const std::vector<Boid>& flock,
                                       std::size_t self_index,
                                       SimConfig const& config) const
{
  phy::Vector2D separation{0, 0};

  for (std::size_t i{0}; i < flock.size(); ++i) {
    if (i == self_index) {
      continue;
    }
    const auto& b = flock[i];

    double dist{position_.distance(b.get_position())};

    if (dist < config.separation_radius) {
      separation -= (b.get_position() - position_) * config.separation_factor;
    }
  }

  return separation;
}

phy::Vector2D Boid::compute_alignment(const std::vector<Boid>& flock,
                                      std::size_t self_index,
                                      SimConfig const& config) const
{
  phy::Vector2D mean_velocity{0, 0};
  int count{0};

  for (std::size_t i{0}; i < flock.size(); ++i) {
    if (i == self_index) {
      continue;
    }
    const auto& b = flock[i];
    if (position_.distance(b.get_position()) < config.visual_range) {
      mean_velocity += b.get_velocity();
      ++count;
    }
  }

  if (count == 0) {
    return {0, 0};
  }

  mean_velocity = mean_velocity / count;

  return (mean_velocity - velocity_) * config.alignment_factor;
}
phy::Vector2D Boid::compute_cohesion(const std::vector<Boid>& flock,
                                     std::size_t self_index,
                                     SimConfig const& config) const
{
  phy::Vector2D center_of_mass{0, 0};
  int count{0};

  for (std::size_t i{0}; i < flock.size(); ++i) {
    if (i == self_index) {
      continue;
    }
    const auto& b = flock[i];

    if (position_.distance(b.get_position()) < config.visual_range) {
      center_of_mass += b.get_position();
      ++count;
    }
  }

  if (count == 0) {
    return {0, 0};
  }

  center_of_mass = center_of_mass / count;

  return (center_of_mass - position_) * config.cohesion_factor;
}

void Boid::limit_velocity()
{
  double speed{velocity_.norm()};

  if (speed > max_vel_) {
    velocity_ *= (max_vel_ / speed);
  } else if (speed < min_vel_ && speed > 0) {
    velocity_ *= (min_vel_ / speed);
  }
}

void Boid::update(const std::vector<Boid>& flock, std::size_t self_index,
                  SimConfig const& config)
{
  phy::Vector2D separation{compute_separation(flock, self_index, config)};

  phy::Vector2D alignment{compute_alignment(flock, self_index, config)};

  phy::Vector2D cohesion{compute_cohesion(flock, self_index, config)};

  velocity_ += separation + alignment + cohesion;

  limit_velocity();

  position_ += velocity_ * config.dt;
}

void Boid::apply_toroidal_boundary(SimConfig const& config)
{
  if (position_.x_ >= config.border_width) {
    position_.x_ -= config.border_width;
  }

  if (position_.x_ < 0) {
    position_.x_ += config.border_width;
  }

  if (position_.y_ >= config.border_height) {
    position_.y_ -= config.border_height;
  }

  if (position_.y_ < 0) {
    position_.y_ += config.border_height;
  }
}

void Boid::apply_window_boundary(SimConfig const& config)
{
  if (position_.x_ >= (config.border_width - config.margin)) {
    velocity_.x_ -= config.border_turnfactor;
  }
  if (position_.x_ <= config.margin) {
    velocity_.x_ += config.border_turnfactor;
  }
  if (position_.y_ >= (config.border_height - config.margin)) {
    velocity_.y_ -= config.border_turnfactor;
  }
  if (position_.y_ <= config.margin) {
    velocity_.y_ += config.border_turnfactor;
  }
}

void Boid::apply_force(const phy::Vector2D& force_point,
                       double influence_radius, double interaction_strength)
{
  double dist{position_.distance(force_point)};
  if (dist < influence_radius && dist != 0) {
    velocity_ -= (force_point - position_) * (interaction_strength / dist);
  }
}

void Boid::set_max_vel(double max_v)
{
  max_vel_ = max_v;
}
void Boid::set_min_vel(double min_v)
{
  min_vel_ = min_v;
}

std::vector<Boid> entity_gen(SimConfig const& config)
{
  if (config.is_hunter && config.n_entities < 0) {
    throw std::runtime_error("Invalid number of hunters (must be >= 0).");
  } else if (!config.is_hunter && config.n_entities <= 0) {
    throw std::runtime_error("Invalid number of boids (must be > 0).");
  }

  std::random_device r;
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<double> x{100.0, config.border_width - 100.0};
  std::uniform_real_distribution<double> y{100.0, config.border_height - 100.0};
  std::uniform_real_distribution<double> v{-35.0, 35.0};

  std::vector<Boid> entities;
  entities.reserve(static_cast<size_t>(config.n_entities));

  for (int i{0}; i < config.n_entities; ++i) {
    phy::Vector2D pos{x(eng), y(eng)};
    phy::Vector2D vel{v(eng), v(eng)};

    Boid e{pos, vel};

    if (config.is_hunter) {
      e.set_max_vel(config.max_vel);
      e.set_min_vel(config.min_vel);
    }

    entities.push_back(e);
  }

  return entities;
}

} // namespace boids