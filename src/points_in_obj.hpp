#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdexcept>
#define FAST_OBJ_IMPLEMENTATION
#include "third_party/fast_obj.h"
#include "third_party/json.hpp"
#include "third_party/isInPoly.hpp"
// for convenience
using json = nlohmann::json;

std::vector<std::vector<std::string>> findObjects(const char* objPath, const std::vector<Point> pointList) {
  std::vector<std::vector<std::string>> ret(pointList.size(), std::vector<std::string>(0));
  std::string path = objPath;
  fastObjMesh* m = fast_obj_read(objPath);
  if (!m)
    throw std::invalid_argument("Impossible to read " + path);

  for (unsigned int ii = 0; ii < m->object_count; ii++) { // m->group_count
    const fastObjGroup& grp = m->objects[ii];              // m->groups[ii]
    std::string grp_name = "unnamed_object_" + std::to_string(ii);
    if (grp.name)
      grp_name = grp.name;

    std::vector<Face> polyhedron = {};

    int idx = 0;
    for (unsigned int jj = 0; jj < grp.face_count; jj++) {
      Face polyhedronFace = {};
      unsigned int fv = m->face_vertices[grp.face_offset + jj];
      for (unsigned int kk = 0; kk < fv; kk++) {
        fastObjIndex mi = m->indices[grp.index_offset + idx];
        if (mi.p) {                          //mi.n
          Point vp;
          vp.x = m->positions[3 * mi.p + 0]; //m->normals[3 * mi.n + 0]
          vp.y = m->positions[3 * mi.p + 1]; //m->normals[3 * mi.n + 1]
          vp.z = m->positions[3 * mi.p + 2]; //m->normals[3 * mi.n + 2]
          polyhedronFace.v.push_back(vp);
        }
        idx++;
      }
      polyhedron.push_back(polyhedronFace);
    }

    for (unsigned int pp = 0; pp < pointList.size(); pp++) {
      if (isInPoly2(pointList[pp], polyhedron))
        ret[pp].push_back(grp_name);
    }
  }
  fast_obj_destroy(m);
  return ret;
}

std::string findObjectsJson(const char* objPath, const std::string jsonString) {
  //jsonString sample = "[[0,0,0], [10,0,0]]"
  json pointListJson;
  try {
    pointListJson = json::parse(jsonString);
  } catch (json::exception& e) {
    throw std::invalid_argument("Invalid JSON provided: " + std::string(e.what()));
  }
  std::string invalidJSONMessage = "Invalid JSON provided: Expected an array of arrays containing 3 int representing x, y, z. Es: [[0,0,0], [10,0,0]]. Received " + jsonString;
  if (!pointListJson.is_array())
    throw std::invalid_argument(invalidJSONMessage);

  std::vector<Point> pointList;
  for(json p: pointListJson) {
    if (!p.is_array() || p.size() != 3 || !p[0].is_number() || !p[1].is_number() || !p[2].is_number())
      throw std::invalid_argument(invalidJSONMessage);
    pointList.push_back(Point{
      p[0],
      p[1],
      p[2]
    });
  }

  std::vector<std::vector<std::string>> out = findObjects(objPath, pointList);
  json ret = json::array();
  for (std::vector<std::string> o: out) {
    json r = json::array();
    for (std::string s: o)
      r.push_back(s);
    ret.push_back(r);
  }
  //ret sample = [["unnamed_object_0"],[]]
  return ret.dump();
}