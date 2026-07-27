#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

namespace phy {
struct Vector2D
{
  double x_{0.0};
  double y_{0.0};

  Vector2D operator+=(const Vector2D& vec);

  Vector2D operator-=(const Vector2D& vec);

  Vector2D operator*=(double scalare);

  Vector2D operator/=(double scalare);

  double distance(const Vector2D& vec) const;

  double norm2() const;

  double norm() const;

  double angle() const;
};

Vector2D operator+(Vector2D lhs, const Vector2D& rhs);

Vector2D operator-(Vector2D lhs, const Vector2D& rhs);

Vector2D operator*(Vector2D lhs, const double& scalare);

Vector2D operator*(const double& scalare, Vector2D vec);

Vector2D operator/(Vector2D lhs, const double& scalare);

Vector2D operator/(const double& scalare, Vector2D vec);

} // namespace phy
#endif