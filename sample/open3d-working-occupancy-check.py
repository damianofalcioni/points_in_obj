#Dependencies:
# pip install open3d
#Requirement: 
# OBJ must contain mesh of triangules
import open3d as o3d
mesh = o3d.io.read_triangle_mesh("Sendlinger_Tor_Munich_ZoneOnly_brg.obj")
mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(mesh)
occupancy = scene.compute_occupancy(o3d.core.Tensor([[16.81775, 33.73125, 512.21]], dtype=o3d.core.Dtype.Float32))
print("occupancy", occupancy.numpy())