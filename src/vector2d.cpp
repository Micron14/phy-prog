#include "vector2d.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace boids {

Vector2D& Vector2D::operator+=(Vector2D const& vec)
{
  x_ += vec.x_;
  y_ += vec.y_;
  return *this;
}

Vector2D& Vector2D::operator-=(Vector2D const& vec)
{
  x_ -= vec.x_;
  y_ -= vec.y_;
  return *this;
}

Vector2D& Vector2D::operator*=(double scalare)
{
  x_ *= scalare;
  y_ *= scalare;
  return *this;
}

Vector2D& Vector2D::operator/=(double scalare)
{
  if (scalare == 0.0) {
    throw std::runtime_error("Division by_ zero in Vector2D::operator/=");
  }
  x_ /= scalare;
  y_ /= scalare;
  return *this;
}

Vector2D operator+(Vector2D lhs, Vector2D const& rhs)
{
  lhs += rhs;
  return lhs;
}

Vector2D operator-(Vector2D lhs, Vector2D const& rhs)
{
  lhs -= rhs;
  return lhs;
}

Vector2D operator*(Vector2D lhs, double scalare)
{
  lhs *= scalare;
  return lhs;
}

Vector2D operator*(double scalare, Vector2D vec)
{
  vec *= scalare;
  return vec;
}

Vector2D operator/(Vector2D vec, double scalare)
{
  vec /= scalare;
  return vec;
}

double Vector2D::distance(Vector2D const& vec) const
{
  return std::sqrt((x_ - vec.x_) * (x_ - vec.x_)
                   + (y_ - vec.y_) * (y_ - vec.y_));
}

double Vector2D::norm2() const
{
  return x_ * x_ + y_ * y_;
}

double Vector2D::norm() const
{
  return std::sqrt(x_ * x_ + y_ * y_);
}

double Vector2D::angle() const
{
  double radians{std::atan2(y_, x_)};
  double degrees{radians * (180.0 / std::numbers::pi)};
  return degrees;
}

} // namespace boids