#include "points_in_obj.hpp"

int main(int argc, const char* argv[]) {
  try {
    if (argc != 3)
      throw std::invalid_argument("Invalid number of arguments provided. Expected usage: points_in_obj my_obj_file_path my_array_of_points . Sample: points_in_obj ../sample/sample.obj \"[[0,0,0], [10,0,0]]\"");
    
    const char* path = argv[1];             //"sample.obj";
    const std::string jsonString = argv[2]; //R"([[0,0,0], [10,0,0]])";

    std::string out = findObjectsJson(path, jsonString);
    std::cout << out << std::endl;
  } catch (const std::invalid_argument& e ) {
    json j = {{"error", e.what()}};
    std::cout << j << std::endl;
  } catch (...) {
    printf("{\"error\":\"generic error\"}");
  }
  
  return 0;
}