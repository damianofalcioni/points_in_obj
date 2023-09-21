//https://www.codefull.net/2016/03/cgal-point-in-polyhedron-algorithm/
//NOT IMPLEMENTED YET
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/algorithm.h>
#include <CGAL/Side_of_triangle_mesh.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef CGAL::Side_of_triangle_mesh<Polyhedron, K> Point_inside;

bool pointInside(Polyhedron &polyhedron, Point &query) {
    // Construct AABB tree with a KdTree
    Tree tree(faces(polyhedron).first, faces(polyhedron).second, polyhedron);
    tree.accelerate_distance_queries();
    // Initialize the point-in-polyhedron tester
    Point_inside inside_tester(tree);
    
    // Determine the side and return true if inside!
    return inside_tester(query) == CGAL::ON_BOUNDED_SIDE;
}