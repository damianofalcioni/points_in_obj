/*
source: https://stackoverflow.com/questions/8877872/determining-if-a-point-is-inside-a-polyhedron
*/
#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>

struct Vector {
  double x, y, z;

  Vector operator-(Vector p) const {
    return Vector{x - p.x, y - p.y, z - p.z};
  }

  Vector cross(Vector p) const {
    return Vector{
      y * p.z - p.y * z,
      z * p.x - p.z * x,
      x * p.y - p.x * y
    };
  }

  double dot(Vector p) const {
    return x * p.x + y * p.y + z * p.z;
  }

  double norm() const {
    return std::sqrt(x*x + y*y + z*z);
  }
};

using Point = Vector;

struct Face {
  std::vector<Point> v;

  Vector normal() const {
    assert(v.size() > 2);
    Vector dir1 = v[1] - v[0];
    Vector dir2 = v[2] - v[0];
    Vector n  = dir1.cross(dir2);
    double d = n.norm();
    return Vector{n.x / d, n.y / d, n.z / d};
  }
};

bool isInConvexPoly(Point const& p, std::vector<Face> const& fs) {
  for (Face const& f : fs) {
    Vector p2f = f.v[0] - p;         // f.v[0] is an arbitrary point on f
    double d = p2f.dot(f.normal());
    d /= p2f.norm();                 // for numeric stability

    constexpr double bound = -1e-15; // use 1e15 to exclude boundaries
    if (d < bound)
      return false;
  }

  return true;
}