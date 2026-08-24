#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

namespace bs {
struct Vector2D
{
  double x{0.0};
  double y{0.0};

  Vector2D& operator+=(Vector2D const& vec);

  Vector2D& operator-=(Vector2D const& vec);

  Vector2D& operator*=(double scalare);

  Vector2D& operator/=(double scalare);

  double distance2(Vector2D const& vec) const;

  double distance(Vector2D const& vec) const;

  double norm2() const;

  double norm() const;

  double angle() const;
};

Vector2D operator+(Vector2D lhs, Vector2D const& rhs);

Vector2D operator-(Vector2D lhs, Vector2D const& rhs);

Vector2D operator*(Vector2D lhs, double scalare);

Vector2D operator*(double scalare, Vector2D vec);

Vector2D operator/(Vector2D lhs, double scalare);

} // namespace bs
#endif