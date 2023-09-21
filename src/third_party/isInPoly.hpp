#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>

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

  Vector normal() const {
    assert(v.size() > 2);
    Vector dir1 = v[1] - v[0];
    Vector dir2 = v[2] - v[0];
    Vector n  = dir1.cross(dir2);
    double d = n.norm();
    return Vector{n.x / d, n.y / d, n.z / d};
  }
};

struct Ray {
  Point orig;
  Point dir;
};

//https://stackoverflow.com/questions/8877872/determining-if-a-point-is-inside-a-polyhedron
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

//https://github.com/johnnovak/raytriangle-test/blob/master/cpp/perftest.cpp
float rayTriangleIntersect(Ray const& r, Point const& v0, Point const& v1, Point const& v2) {
  Point v0v1 = v1 - v0;
  Point v0v2 = v2 - v0;
  Point pvec = r.dir.cross(v0v2);
  float det = v0v1.dot(pvec);
  if (det < 0.000001)
    return -INFINITY;
  float invDet = 1.0 / det;
  Point tvec = r.orig - v0;
  float u = tvec.dot(pvec) * invDet;
  if (u < 0 || u > 1)
    return -INFINITY;
  Point qvec = tvec.cross(v0v1);
  float v = r.dir.dot(qvec) * invDet;
  if (v < 0 || u + v > 1)
    return -INFINITY;
  return v0v2.dot(qvec) * invDet;
}

//https://saturncloud.io/blog/algorithm-for-determining-whether-a-point-is-inside-a-3d-mesh/
bool isInPoly(Point const& p, std::vector<Face> const& fs) {
  Ray r;
  r.orig = p;
  r.dir = Point{1000, 0, 0};
  //r.dir = (Point{0, 0, -1} - p).normalize();

  int count = 0;
  for (Face const& f : fs) {

    float t = rayTriangleIntersect(r, f.v[0], f.v[1], f.v[2]);
    if (t>=0) {
      count++;
    }
  }
  if(count % 2 == 0)
    return true;
  else
    return false;
}


//https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm#C++_implementation
//work only on triangular faces
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

//https://saturncloud.io/blog/algorithm-for-determining-whether-a-point-is-inside-a-3d-mesh/
bool isInPoly2(Point const& p, std::vector<Face> const& fs) {
  Ray r;
  r.orig = p;
  r.dir = Point{1000, 0, 0};
  //r.dir = (Point{0, 0, -1} - p).normalize();

  int count = 0;
  for (Face const& f : fs) {
    if (RayIntersectsTriangle(r, f)) {
      count++;
    }
  }
  if(count % 2 == 0)
    return false;
  else
    return true;
}


//https://stackoverflow.com/questions/312328/what-is-the-fastest-way-to-find-the-point-of-intersection-between-a-ray-and-a-po
bool findIntersection(Ray ray, Face const& face) {
  Point plane_normal = (face.v[1] - face.v[0]).cross(face.v[2] - face.v[0]);
  float denominator = (ray.dir - face.v[0]).dot(plane_normal);
  if (denominator == 0) { return false; }
  float ray_scalar = (face.v[0] - ray.orig).dot(plane_normal);
  Point answer = ray.orig + ray.dir.scalarMult(ray_scalar);
  Point test_line = answer - face.v[0];
  Point test_axis = plane_normal.cross(test_line);
  bool point_is_inside = false;
  Point test_point = face.v[1] - answer;
  bool prev_point_ahead = test_line.dot(test_point) > 0;
  bool prev_point_above = test_axis.dot(test_point) > 0;
  bool this_point_ahead, this_point_above;
  int index = 2;
  while (index < face.v.size()) {
    test_point = face.v[index] - answer;
    this_point_ahead = test_line.dot(test_point) > 0;
    if (prev_point_ahead || this_point_ahead) {
      this_point_above = test_axis.dot(test_point) > 0;
      if (prev_point_above == !this_point_above) {
        point_is_inside = !point_is_inside;
      }
    }
    prev_point_ahead = this_point_ahead;
    prev_point_above = this_point_above;
    index++;
  }
  return point_is_inside;
}

//https://saturncloud.io/blog/algorithm-for-determining-whether-a-point-is-inside-a-3d-mesh/
bool isInPoly3(Point const& p, std::vector<Face> const& fs) {
  Ray r;
  r.orig = p;
  r.dir = Point{1000, 0, 0};
  //r.dir = (Point{0, 0, -1} - p).normalize();

  int count = 0;
  for (Face const& f : fs) {
    if (findIntersection(r, f)) {
      count++;
    }
  }
  if(count % 2 == 0)
    return false;
  else
    return true;
}