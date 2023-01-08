import copy
import numpy as np
import open3d
import open3d.visualization
from scipy.spatial.transform import Rotation

path = "./Cone.stl"
# path = "./Partial Cone.stl"
mesh = open3d.io.read_triangle_mesh(path)
mesh.scale(1e-3, np.array([0.0, 0.0, 0.0]))
original_mesh = copy.deepcopy(mesh)

R = mesh.get_rotation_matrix_from_xyz(
    (np.deg2rad(100.0), np.deg2rad(130.0), np.deg2rad(200.0))
)

mesh.rotate(R)

# pcl = open3d.geometry.PointCloud()
# pcl.points = mesh.vertices
# pcl.colors = mesh.vertex_colors
# pcl.normals = mesh.vertex_normals

mesh_pcl = mesh.sample_points_uniformly(100)

pcl = open3d.geometry.PointCloud()
pcl.points = copy.deepcopy(mesh_pcl.points)

bb = pcl.get_oriented_bounding_box()

pca_R = copy.deepcopy(bb.R)
pca_center = bb.get_center()
pca_T = np.eye(4)
pca_T[:3, :3] = pca_R
pca_T[:3, 3] = pca_center

mesh_axis = open3d.geometry.TriangleMesh.create_box(
    width=0.005, height=0.005, depth=1.0
)
mesh_axis.translate(-mesh_axis.get_center())

pca_axis = open3d.geometry.TriangleMesh.create_box(width=0.005, height=0.005, depth=1.0)
pca_axis.translate(-pca_axis.get_center())
pca_axis.transform(pca_T)

Rmat = Rotation.from_matrix(pca_R)
print(Rmat.as_euler("xyz", degrees=True))
print(Rotation.from_matrix(R).as_euler("xyz", degrees=True))

print(bb)
# help(pcl)

vis = open3d.visualization.Visualizer()
vis.create_window()

opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])

vis.add_geometry(original_mesh)
vis.add_geometry(pcl)
vis.add_geometry(bb)
vis.add_geometry(pca_axis)
vis.add_geometry(mesh_axis)
vis.run()
vis.destroy_window()
