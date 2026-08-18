#include "boid.hpp"
#include "simconfig.hpp"
#include "vector2d.hpp"

#include <vector>

namespace bs {

Vector2D const& Boid::get_position() const
{
  return position_;
}

Vector2D const& Boid::get_velocity() const
{
  return velocity_;
}

Vector2D Boid::compute_separation(std::vector<Boid> const& flock,
                                  SimConfig const& config) const
{
  Vector2D separation{0, 0};

  for (auto const& b : flock) {
    if (&b == this) {
      continue;
    }
    double dist{position_.distance(b.get_position())};
    if (dist < config.separation_radius) {
      separation -= (b.get_position() - position_) * config.separation_factor;
    }
  }

  return separation;
}

Vector2D Boid::compute_alignment(std::vector<Boid> const& flock,
                                 SimConfig const& config) const
{
  Vector2D mean_velocity{0, 0};
  int count{0};

  for (auto const& b : flock) {
    if (&b == this) {
      continue;
    }
    double dist{position_.distance(b.get_position())};
    if (dist < config.visual_range) {
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

Vector2D Boid::compute_cohesion(std::vector<Boid> const& flock,
                                SimConfig const& config) const
{
  Vector2D center_of_mass{0, 0};
  int count{0};

  for (auto const& b : flock) {
    if (&b == this) {
      continue;
    }
    double dist{position_.distance(b.get_position())};

    if (dist < config.visual_range) {
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

Boid Boid::update(std::vector<Boid> const& flock, SimConfig const& config) const
{
  Boid next_state = *this;

  Vector2D separation{compute_separation(flock, config)};
  Vector2D alignment{compute_alignment(flock, config)};
  Vector2D cohesion{compute_cohesion(flock, config)};

  next_state.velocity_ += separation + alignment + cohesion;
  next_state.limit_velocity();
  next_state.position_ += next_state.velocity_ * config.dt;

  return next_state;
}

void Boid::apply_toroidal_boundary(SimConfig const& config)
{
  if (position_.x >= config.border_width) {
    position_.x -= config.border_width;
  }

  if (position_.x < 0) {
    position_.x += config.border_width;
  }

  if (position_.y >= config.border_height) {
    position_.y -= config.border_height;
  }

  if (position_.y < 0) {
    position_.y += config.border_height;
  }
}

void Boid::apply_window_boundary(SimConfig const& config)
{
  if (position_.x >= (config.border_width - config.margin)) {
    velocity_.x -= config.border_turnfactor;
  }
  if (position_.x <= config.margin) {
    velocity_.x += config.border_turnfactor;
  }
  if (position_.y >= (config.border_height - config.margin)) {
    velocity_.y -= config.border_turnfactor;
  }
  if (position_.y <= config.margin) {
    velocity_.y += config.border_turnfactor;
  }
}

void Boid::apply_force(Vector2D const& force_point, double influence_radius,
                       double interaction_strength)
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

} // namespace bs