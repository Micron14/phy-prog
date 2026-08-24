#include "vector2d.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace bs {

Vector2D& Vector2D::operator+=(Vector2D const& vec)
{
  x += vec.x;
  y += vec.y;
  return *this;
}

Vector2D& Vector2D::operator-=(Vector2D const& vec)
{
  x -= vec.x;
  y -= vec.y;
  return *this;
}

Vector2D& Vector2D::operator*=(double scalare)
{
  x *= scalare;
  y *= scalare;
  return *this;
}

Vector2D& Vector2D::operator/=(double scalare)
{
  if (scalare == 0.0) {
    throw std::runtime_error("Division by zero in Vector2D::operator/=");
  }
  x /= scalare;
  y /= scalare;
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

double Vector2D::norm2() const
{
  return x * x + y * y;
}

double Vector2D::norm() const
{
  return std::sqrt(norm2());
}

double Vector2D::distance2(Vector2D const& vec) const
{
  return (*this - vec).norm2();
}

double Vector2D::distance(Vector2D const& vec) const
{
  return std::sqrt(distance2(vec));
}

double Vector2D::angle() const
{
  double radians{std::atan2(y, x)};
  double degrees{radians * (180.0 / std::numbers::pi)};
  return degrees;
}

} // namespace bs