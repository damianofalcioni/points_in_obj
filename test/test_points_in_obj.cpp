#include "../src/points_in_obj.hpp"

std::string getMeanPoints(const char* objPath) {
  std::string path = objPath;
  fastObjMesh* m = fast_obj_read(objPath);
  if (!m)
    throw std::invalid_argument("Impossible to read " + path);

  std::vector<Point> meanPoints = {};
  for (unsigned int ii = 0; ii < m->object_count; ii++) {
    Point mean;
    int pointCount = 0;
    const fastObjGroup& grp = m->objects[ii];
    int idx = 0;
    for (unsigned int jj = 0; jj < grp.face_count; jj++) {
      unsigned int fv = m->face_vertices[grp.face_offset + jj];
      for (unsigned int kk = 0; kk < fv; kk++) {
        fastObjIndex mi = m->indices[grp.index_offset + idx];
        if (mi.p) {
          Point vp;
          vp.x = m->positions[3 * mi.p + 0];
          vp.y = m->positions[3 * mi.p + 1];
          vp.z = m->positions[3 * mi.p + 2];
          mean = mean + vp;
          pointCount++;
        }
        idx++;
      }
    }
    if (pointCount == 0)
      throw std::invalid_argument("No points in the provided OBJ file");
    mean.x = mean.x / pointCount;
    mean.y = mean.y / pointCount;
    mean.z = mean.z / pointCount;
    meanPoints.push_back(mean);
  }
  fast_obj_destroy(m);

  json ret = json::array();
  for (Point p: meanPoints) {
    json a = {p.x, p.y, p.z};
    ret.push_back(a);
  }
  std::string rets = ret.dump();
  printf("Mean Points: %s\n", rets.c_str());
  return rets;
}

int main(int argc, const char* argv[]) {
  try {
    std::vector<const char*> paths = {
        "../sample/sample.obj",
        "../sample/B9035-USM-TLI-ZZZ-MOD-ST-00000-S03_onlyzones_brg.obj",
        "../sample/Sendlinger_Tor_Munich_ZoneOnly_brg.obj",
        "../sample/Sendlinger_Tor_Munich_ZoneOnly_brg_notri.obj"
    };
    for (const char* path: paths) {
      printf("TEST File: %s\n", path);
      printf("Matching Objects: %s\n\n", findObjectsJson(path, getMeanPoints(path)).c_str());
    }
  } catch (const std::invalid_argument& e ) {
    printf("Error: %s\n", e.what());
  } catch (...) {
    printf("Error: generic error");
  }
  
  return 0;
}