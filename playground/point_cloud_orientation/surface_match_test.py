import os
import open3d
import cv2 as cv
import numpy as np
import time

N = 2
modelname = "simple_cone"
scenename = "cone"

detector = cv.ppf_match_3d_PPF3DDetector(0.025, 0.05)

print('Loading model...')
pc = cv.ppf_match_3d.loadPLYSimple("data/%s.ply" % modelname, 1)


print('Training...')
t0 = time.time()
detector.trainModel(pc)
t1 = time.time()
print(f"Took {t1 - t0} seconds")

print('Loading scene...')
pcTest = cv.ppf_match_3d.loadPLYSimple("data/%s.ply" % scenename, 1)

print('Matching...')
results = detector.match(pcTest, 1.0/40.0, 0.05)

print('Performing ICP...')
icp = cv.ppf_match_3d_ICP(100)
_, results = icp.registerModelToScene(pc, pcTest, results[:N])

print("Poses: ")
out_path = "%sPCTrans.ply" % modelname
for i, result in enumerate(results):
    #result.printPose()
    print("\n-- Pose to Model Index %d: NumVotes = %d, Residual = %f\n%s\n" % (result.modelIndex, result.numVotes, result.residual, result.pose))
    if i == 0:
        pct = cv.ppf_match_3d.transformPCPose(pc, result.pose)
        cv.ppf_match_3d.writePLY(pct, out_path)

assert os.path.isfile(out_path)

mesh = open3d.io.read_point_cloud(out_path)
cone_mesh = open3d.io.read_triangle_mesh("data/cone.stl")

vis = open3d.visualization.Visualizer()
vis.create_window()

opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])
opt.show_coordinate_frame = True

vis.add_geometry(mesh)
vis.add_geometry(cone_mesh)

vis.run()
vis.destroy_window()
