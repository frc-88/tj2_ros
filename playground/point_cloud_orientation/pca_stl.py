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

np.random.seed(100)

R = mesh.get_rotation_matrix_from_xyz(
    (
        np.deg2rad(np.random.random() * 360.0),
        np.deg2rad(np.random.random() * 360.0),
        np.deg2rad(np.random.random() * 360.0),
    )
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
rotate_up = mesh.get_rotation_matrix_from_xyz(
    (np.deg2rad(90.0), np.deg2rad(90.0), np.deg2rad(90.0))
)
pca_T = np.eye(4)
pca_T[:3, :3] = pca_R @ rotate_up
pca_T[:3, 3] = pca_center

mesh_axis = open3d.geometry.TriangleMesh.create_box(
    width=0.005, height=0.005, depth=1.0
)
mesh_axis.translate(-mesh_axis.get_center())

pca_axis = open3d.geometry.TriangleMesh.create_box(width=0.005, height=0.005, depth=1.0)
# pca_axis.rotate(rotate_up)
pca_axis.translate(-pca_axis.get_center())
pca_axis.transform(pca_T)

pca_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
pca_frame.transform(pca_T)

print(Rotation.from_matrix(pca_R).as_euler("xyz", degrees=True))
print(Rotation.from_matrix(R).as_euler("xyz", degrees=True))

print(bb)
# help(pcl)

vis = open3d.visualization.Visualizer()
vis.create_window()

opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])
opt.show_coordinate_frame = True

vis.add_geometry(original_mesh)
vis.add_geometry(pcl)
vis.add_geometry(bb)
vis.add_geometry(pca_axis)
vis.add_geometry(mesh_axis)
vis.add_geometry(pca_frame)
vis.run()
vis.destroy_window()
