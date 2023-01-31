import os
import argparse
import numpy as np
import open3d


parser = argparse.ArgumentParser(description="mesh_to_pointcloud", add_help=True)
parser.add_argument("mesh", help="path to mesh file")
parser.add_argument("--scale", default=1e-3, help="unit conversion (default: 1e-3, mm -> m)")
parser.add_argument("--num_points", default=1000, help="number of points to sample")
args = parser.parse_args()

input_path = args.mesh
assert os.path.isfile(input_path)
output_path = os.path.splitext(input_path)[0] + ".ply"

mesh = open3d.io.read_triangle_mesh(input_path)
print("Read", mesh)
mesh.scale(args.scale, np.array([0.0, 0.0, 0.0]))

# open3d.io.write_triangle_mesh(output_path, mesh, write_ascii=True)
# mesh_pcl = mesh.sample_points_uniformly(args.num_points)
# wrote_mesh = open3d.io.read_triangle_mesh(output_path)
mesh_pcl = mesh.sample_points_poisson_disk(args.num_points)
open3d.io.write_point_cloud(output_path, mesh_pcl, write_ascii=True)
wrote_mesh = open3d.io.read_point_cloud(output_path)

print("Wrote", wrote_mesh)


print(f"Wrote to {output_path}")

vis = open3d.visualization.Visualizer()
vis.create_window()

opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])
opt.show_coordinate_frame = True

vis.add_geometry(wrote_mesh)

vis.run()
vis.destroy_window()
