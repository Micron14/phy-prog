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

    if (position_.distance2(b.get_position())
        < config.separation_radius * config.separation_radius) {
      separation -= (b.get_position() - position_) * config.separation_factor;
    }
  }

  return separation;
}

Vector2D Boid::compute_alignment(std::vector<Boid> const& flock,
                                 SimConfig const& config) const
{
  Vector2D sum_velocity{0, 0};
  int count{0};

  for (auto const& b : flock) {
    if (&b == this) {
      continue;
    }

    if (position_.distance2(b.get_position())
        < config.visual_range * config.visual_range) {
      sum_velocity += b.get_velocity();
      ++count;
    }
  }

  if (count == 0) {
    return {0, 0};
  }

  return (sum_velocity / count - velocity_) * config.alignment_factor;
}

Vector2D Boid::compute_cohesion(std::vector<Boid> const& flock,
                                SimConfig const& config) const
{
  Vector2D sum_position{0, 0};
  int count{0};

  for (auto const& b : flock) {
    if (&b == this) {
      continue;
    }

    if (position_.distance2(b.get_position())
        < config.visual_range * config.visual_range) {
      sum_position += b.get_position();
      ++count;
    }
  }

  if (count == 0) {
    return {0, 0};
  }

  return (sum_position / count - position_) * config.cohesion_factor;
}

void Boid::limit_velocity()
{
  double speed2{velocity_.norm2()};

  if (speed2 > max_vel_ * max_vel_) {
    velocity_ *= (max_vel_ / std::sqrt(speed2));
  } else if (speed2 < min_vel_ * min_vel_ && speed2 > 0.0) {
    velocity_ *= (min_vel_ / std::sqrt(speed2));
  }
}

Boid Boid::update(std::vector<Boid> const& flock, SimConfig const& config) const
{
  Boid next_state{*this};

  next_state.velocity_ += compute_separation(flock, config)
                        + compute_alignment(flock, config)
                        + compute_cohesion(flock, config);
  next_state.limit_velocity();
  next_state.position_ += next_state.velocity_ * config.dt;

  return next_state;
}

void Boid::apply_toroidal_boundary(SimConfig const& config)
{
  if (position_.x >= config.border_width) {
    position_.x -= config.border_width;
  } else if (position_.x < 0) {
    position_.x += config.border_width;
  }

  if (position_.y >= config.border_height) {
    position_.y -= config.border_height;
  } else if (position_.y < 0) {
    position_.y += config.border_height;
  }
}

void Boid::apply_window_boundary(SimConfig const& config)
{
  if (position_.x >= (config.border_width - config.margin)) {
    velocity_.x -= config.border_turnfactor;
  } else if (position_.x <= config.margin) {
    velocity_.x += config.border_turnfactor;
  }

  if (position_.y >= (config.border_height - config.margin)) {
    velocity_.y -= config.border_turnfactor;
  } else if (position_.y <= config.margin) {
    velocity_.y += config.border_turnfactor;
  }
}

void Boid::apply_force(Vector2D const& force_point, double influence_radius,
                       double interaction_strength)
{
  Vector2D delta{force_point - position_};
  double dist2{delta.norm2()};

  if (dist2 < influence_radius * influence_radius && dist2 > 0.0) {
    velocity_ -= delta * (interaction_strength / std::sqrt(dist2));
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