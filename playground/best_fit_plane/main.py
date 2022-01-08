# https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

def best_fit_plane(points):
    # find the best-fitting plane for the test points
    # subtract out the centroid and take the SVD
    centroid = np.mean(points, axis=1, keepdims=True)
    u, s, vh = np.linalg.svd(points - centroid)

    normal = u[:, 2]
    point = centroid.T[0]

    d = -normal.dot(point)
    a, b, c = normal

    return a, b, c, d, centroid

def get_plane_mesh(a, b, c, d, x_lim=(-1.0, 1.0), y_lim=(-1.0, 1.0)):
    X, Y = np.meshgrid(x_lim, y_lim)
    if c != 0:
        Z = -(a * X + b * Y + d) / c

    elif b != 0:
        Z = -(a * X + c * Y + d) / b
        X, Y, Z = X, Z, Y

    else:
        Z = -(b * X + c * Y + d) / a
        X, Y, Z = Z, X, Y
    return X, Y, Z


def main():
    np.random.seed(8000)

    # generate some random test points 
    m = 10 # number of points
    delta = 1.0 # size of random displacement
    origin = np.random.rand(3, 1) # random origin for the plane
    # origin = np.zeros((3, 1))
    basis = np.random.rand(3, 2) # random basis vectors for the plane
    coefficients = np.random.rand(2, m) # random coefficients for points on the plane

    # generate random points on the plane and add random displacement
    points = basis @ coefficients \
            + np.tile(origin, (1, m)) \
            + delta * np.random.rand(3, m)

    a, b, c, d, centeroid = best_fit_plane(points)
    X, Y, Z = get_plane_mesh(a, b, c, d, 
        (np.min(points[0]), np.max(points[0])),
        (np.min(points[1]), np.max(points[1]))
    )

    # fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter3D(points[0], points[1], points[2], s=1)
    # ax.scatter3D(centeroid[0], centeroid[1], centeroid[2], 'x', s=100)
    ax.plot_surface(X, Y, Z, alpha=0.5)
    plt.show()

main()
