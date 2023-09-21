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












//------- source Lilis Georgios
double dot3D(double *a, double *b) {
  double dp = 0.0;
  dp = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return dp;
}

double *cross3D(double *a, double *b) {
  double *cp = (double *)malloc(3 * sizeof(double));
  cp[0] = a[1] * b[2] - a[2] * b[1];
  cp[1] = a[2] * b[0] - a[0] * b[2];
  cp[2] = a[0] * b[1] - a[1] * b[0];
  return cp;
}

double norm3D(double *a) {
  double nrm = 0.0;
  nrm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  return nrm;
}

bool lineSegment_Intersects_Plane(double px, double py, double pz,
                                  double mx, double my, double mz,
                                  double p0x, double p0y, double p0z,
                                  double p1x, double p1y, double p1z,
                                  double p2x, double p2y, double p2z) {
  bool intersects = false;
  double *P0_Pp = (double *)malloc(3 * sizeof(double));
  double *P0_Pm = (double *)malloc(3 * sizeof(double));
  double *P0_P1 = (double *)malloc(3 * sizeof(double));
  double *P0_P2 = (double *)malloc(3 * sizeof(double));
  P0_Pp[0] = px - p0x;
  P0_Pp[1] = py - p0y;
  P0_Pp[2] = pz - p0z;
  P0_Pm[0] = mx - p0x;
  P0_Pm[1] = my - p0y;
  P0_Pm[2] = mz - p0z;
  P0_P1[0] = p1x - p0x;
  P0_P1[1] = p1y - p0y;
  P0_P1[2] = p1z - p0z;
  P0_P2[0] = p2x - p0x;
  P0_P2[1] = p2y - p0y;
  P0_P2[2] = p2z - p0z;
  double *C12 = cross3D(P0_P1, P0_P2);
  if (dot3D(P0_Pp, C12) * dot3D(P0_Pm, C12) < 0)
    intersects = true;
  free(P0_Pm);
  free(P0_Pp);
  free(P0_P1);
  free(P0_P2);
  free(C12);
  return intersects;
}

double *point_where_Linesegment_Intersects_Plane(double px, double py, double pz,
                                                 double mx, double my, double mz,
                                                 double p0x, double p0y, double p0z,
                                                 double p1x, double p1y, double p1z,
                                                 double p2x, double p2y, double p2z) {
  double *t = (double *)malloc(3 * sizeof(double));
  double *P0_P1 = (double *)malloc(3 * sizeof(double));
  double *P0_P2 = (double *)malloc(3 * sizeof(double));
  P0_P1[0] = p1x - p0x;
  P0_P1[1] = p1y - p0y;
  P0_P1[2] = p1z - p0z;
  P0_P2[0] = p2x - p0x;
  P0_P2[1] = p2y - p0y;
  P0_P2[2] = p2z - p0z;
  double *C = cross3D(P0_P1, P0_P2);
  double d = (-1) * (p0x * C[0] + p0y * C[1] + p0z * C[2]);
  // Here if there is intersection the denominator (C[0]*(px-mx) + C[1]*(py-my) + C[2]*(pz-mz)) cannot be zero !!!
  double a = ((-1) * (C[0] * mx + C[1] * my + C[2] * mz)) / (C[0] * (px - mx) + C[1] * (py - my) + C[2] * (pz - mz));
  t[0] = mx + a * (px - mx); // tx
  t[1] = my + a * (py - my); // ty
  t[2] = mz + a * (pz - mz); // tz
  free(P0_P1);
  free(P0_P2);
  free(C);
  return t;
}

bool Point3D_Inside_Triangle3D(double tx, double ty, double tz,
                               double p0x, double p0y, double p0z,
                               double p1x, double p1y, double p1z,
                               double p2x, double p2y, double p2z) {
  double *P0_Pt = (double *)malloc(3 * sizeof(double));
  double *P0_P1 = (double *)malloc(3 * sizeof(double));
  double *P0_P2 = (double *)malloc(3 * sizeof(double));
  P0_Pt[0] = tx - p0x;
  P0_Pt[1] = ty - p0y;
  P0_Pt[2] = tz - p0z;
  P0_P1[0] = p1x - p0x;
  P0_P1[1] = p1y - p0y;
  P0_P1[2] = p1z - p0z;
  P0_P2[0] = p2x - p0x;
  P0_P2[1] = p2y - p0y;
  P0_P2[2] = p2z - p0z;
  double *P12 = cross3D(P0_P1, P0_P2);
  double A12 = dot3D(P12, P12);
  bool is_inside = false;
  if ((A12 > 0.0) && (norm3D(P0_Pt) > 0.0) && (norm3D(P0_P1) > 0.0) && (norm3D(P0_P2) > 0.0)) {
    double *DP1_DP = cross3D(P0_P1, P0_Pt);
    double *DP_DP2 = cross3D(P0_Pt, P0_P2);
    double gamma = dot3D(DP1_DP, P12) / A12;
    double beta = dot3D(DP_DP2, P12) / A12;
    double alpha = 1.0 - beta - gamma;
    free(DP1_DP);
    free(DP_DP2);
    is_inside = ((0.0 <= alpha) && (alpha <= 1.0) &&
                 (0.0 <= beta) && (beta <= 1.0) &&
                 (0.0 <= gamma) && (gamma <= 1.0));
  }
  free(P12);
  free(P0_Pt);
  free(P0_P1);
  free(P0_P2);
  return is_inside;
}



bool isInPoly4(Point const& p, std::vector<Face> const& fs) {
  Point m = Point{-100000, -100000, -100000};
  int count = 0;
  for (Face const& f : fs) {
    if (lineSegment_Intersects_Plane(p.x, p.y, p.z, 
                                      m.x, m.y, m.z, 
                                      f.v[0].x, f.v[0].y, f.v[0].z,
                                      f.v[1].x, f.v[1].y, f.v[1].z,
                                      f.v[2].x, f.v[2].y, f.v[2].z)) {
      double* t = point_where_Linesegment_Intersects_Plane(p.x, p.y, p.z, 
                                                    m.x, m.y, m.z, 
                                                    f.v[0].x, f.v[0].y, f.v[0].z,
                                                    f.v[1].x, f.v[1].y, f.v[1].z,
                                                    f.v[2].x, f.v[2].y, f.v[2].z);
      if(Point3D_Inside_Triangle3D(t[0], t[1], t[2],
                                f.v[0].x, f.v[0].y, f.v[0].z,
                                f.v[1].x, f.v[1].y, f.v[1].z,
                                f.v[2].x, f.v[2].y, f.v[2].z))
        count++;
    }
  }
  if(count % 2 == 0)
    return false;
  else
    return true;
}
