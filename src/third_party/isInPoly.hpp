#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>

enum CheckMethod {
  RAY, SEGMENT
};

struct Vector {
  double x, y, z;

  Vector operator-(Vector p) const {
    return Vector{x - p.x, y - p.y, z - p.z};
  }

  Vector operator+(Vector p) const {
    return Vector{x + p.x, y + p.y, z + p.z};
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

  Vector normalize() {
    double l = norm();
    return Vector{x/l, y/l, z/l};
  }

  Vector scalarMult(float a) const {
    return Vector{a*x, a*y, a*z};
  }
};

using Point = Vector;

struct Face {
  std::vector<Point> v;

  double norm() const {
    assert(v.size() > 2);
    Vector dir1 = v[1] - v[0];
    Vector dir2 = v[2] - v[0];
    Vector n  = dir1.cross(dir2);
    return n.norm();
  }

  Vector normal() const {
    assert(v.size() > 2);
    Vector dir1 = v[1] - v[0];
    Vector dir2 = v[2] - v[0];
    Vector n  = dir1.cross(dir2);
    return n.normalize();
  }
};

struct Ray {
  Point orig;
  Point dir;
};

//https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm#C++_implementation
bool RayIntersectsTriangle(Ray const& ray, Face const& face) {
  Point rayOrigin = ray.orig;
  Point rayVector = ray.dir;
  const float EPSILON = 0.0000001;
  Point vertex0 = face.v[0];
  Point vertex1 = face.v[1];
  Point vertex2 = face.v[2];
  Point edge1, edge2, h, s, q;
  float a, f, u, v;
  edge1 = vertex1 - vertex0;
  edge2 = vertex2 - vertex0;
  h = rayVector.cross(edge2);
  a = edge1.dot(h);
  if (a > -EPSILON && a < EPSILON)
    return false;    // This ray is parallel to this triangle.
  f = 1.0 / a;
  s = rayOrigin - vertex0;
  u = f * s.dot(h);
  if (u < 0.0 || u > 1.0)
    return false;
  q = s.cross(edge1);
  v = f * rayVector.dot(q);
  if (v < 0.0 || u + v > 1.0)
    return false;
  // At this stage we can compute t to find out where the intersection point is on the line.
  float t = f * edge2.dot(q);
  if (t > EPSILON) // ray intersection
    return true;
  else // This means that there is a line intersection but not a ray intersection.
    return false;
}

//source: Lilis Georgios
bool Segment_Intersect_Triangle_Barycenter(Point p, Point m, Face f) {
  //check if line segment intersects plane
  Point P0_Pp = p - f.v[0];
  Point P0_Pm = m - f.v[0];
  Point P0_P1 = f.v[1] - f.v[0];
  Point P0_P2 = f.v[2] - f.v[0];
  Point C12 = P0_P1.cross(P0_P2);
  /*
  Point C12N = C12.normalize();
  Point c; // what is c?
  Point insideP = c - C12N.scalarMult(0.0001);
  Point outsideP = c + C12N.scalarMult(0.0001);
  */
  if (P0_Pp.dot(C12) * P0_Pm.dot(C12) >= 0)
    return false;
  //calculate point where line segment intersects Plane
  double d = (-1.0) * (f.v[0].x * C12.x + f.v[0].y * C12.y + f.v[0].z * C12.z);
  double a = ((-1.0) * (d + C12.x * m.x + C12.y * m.y + C12.z * m.z)) / (C12.x * (p.x - m.x) + C12.y * (p.y - m.y) + C12.z * (p.z - m.z));
  Point t = Point{
    m.x + a * (p.x - m.x),
    m.y + a * (p.y - m.y),
    m.z + a * (p.z - m.z)
  };
  //check if intersection point is inside the triangle
  Point P0_Pt = t - f.v[0];
  double A12 = C12.dot(C12);
  if (A12 > 0.0 && P0_Pt.norm() > 0.0 && P0_P1.norm() > 0.0 && P0_P2.norm() > 0.0) {
    Point DP1_DP = P0_P1.cross(P0_Pt);
    Point DP_DP2 = P0_Pt.cross(P0_P2);
    double gamma = DP1_DP.dot(C12) / A12;
    double beta = DP_DP2.dot(C12) / A12;
    double alpha = 1.0 - beta - gamma;
    return ((0.0 <= alpha) && (alpha <= 1.0) && (0.0 <= beta) && (beta <= 1.0) && (0.0 <= gamma) && (gamma <= 1.0));
  }
  return false;
}

//https://saturncloud.io/blog/algorithm-for-determining-whether-a-point-is-inside-a-3d-mesh/
bool isInPoly(Point const& p, std::vector<Face> const& fs, Point minPoint, CheckMethod method) {
  Point m = minPoint - Point{100.0, 100.0, 100.0};
  int count = 0;
  if (method == RAY) {
    Ray r = Ray{p, m};
    for (Face const& f : fs) {
      if (RayIntersectsTriangle(r, f)) {
        count++;
      }
    }
  } else {
    for (Face const& f : fs) {
      if (Segment_Intersect_Triangle_Barycenter(p, m, f)) {
        count++;
      }
    }
  }
  if(count % 2 == 0)
    return false;
  else
    return true;
}

//https://stackoverflow.com/questions/8877872/determining-if-a-point-is-inside-a-polyhedron
bool isInPolyConvex(Point const& p, std::vector<Face> const& fs) {
  for (Face const& f : fs) {
    Vector p2f = f.v[0] - p;
    double d = p2f.dot(f.normal());
    d /= p2f.norm();
    constexpr double bound = -1e-15;
    if (d < bound)
      return false;
  }
  return true;
}