# Points in OBJ
The application take as input a Wavefront OBJ file and an array of points, and return the objects that contain each point

### Buid
```
cmake .
cmake --build .
```

## Run
```
points_in_obj ./sample/sample.obj "[[0,0,0],[10,0,0]]"
```
output:
```
[["unnamed_object_0"],[]]
```

## User Manual
Expected usage: `points_in_obj _my_obj_file_path_ _my_array_of_points_` where:
- `_my_obj_file_path_`   : is the path of your OBJ file. A sample is available in the `sample` folder
- `_my_array_of_points_` : is JSON array containg `point`, where a `point` is defined as a JSON array of 3 numbers corresponding to x, y, z coordinates (eg. `[[0,0,0],[10,0,0]]`)

The application return a JSON array of `object_list`, one for each `point` in the input `_my_array_of_points_`, where `object_list` is a JSON array of object names in the OBJ file, that contain the `point` (eg. `[["unnamed_object_0"],[]]` means that the point `[0,0,0]` is included in the object `unnamed_object_0`, while the point `[10,0,0]` is not inside any objects).

In case of errors it return a JSON object in the form:
```
{
  "error": "description of the error"
}
```