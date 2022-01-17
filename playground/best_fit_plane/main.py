# https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
import numpy as np
import math
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.transform import Rotation as R

def best_fit_plane(points):
    # find the best-fitting plane for the test points
    # subtract out the centroid and take the SVD
    # centroid = np.mean(points, axis=0, keepdims=True)
    centroid = np.array([np.mean(points[:, 0]), np.mean(points[:, 1]), np.mean(points[:, 2])])
    
    u, s, vh = np.linalg.svd((points - centroid).T)

    normal = u[:, 2]
    point = centroid

    d = -normal.dot(point)
    a, b, c = normal

    return a, b, c, d, centroid

def get_plane_mesh(a, b, c, d, x_lim=(-1.0, 1.0), y_lim=(-1.0, 1.0)):
    # Ax + By + Cz = d

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

def plane_to_quat(a, b, c):
    vector = np.array([a, b, c])
    null_vector = np.array([0.0, 0.0, 1.0])
    cross = np.cross(null_vector, np.array(vector))
    cross_mag = np.linalg.norm(cross)
    dot = np.dot(null_vector, vector)
    theta = math.atan(cross_mag / dot)
    axis_n = cross / cross_mag
    quat_xyz = axis_n * np.sin(theta / 2.0)
    w = np.cos(theta / 2.0)
    x = quat_xyz[0]
    y = quat_xyz[1]
    z = quat_xyz[2]
    return np.array([w, x, y, z])


def svd_method():
    np.random.seed(8000)

    A = np.array([3, 1, 1])
    B = np.array([1, 4, 2])
    C = np.array([1, 3, 4])
    points = np.array([A, B, C])

    a, b, c, d, centeroid = best_fit_plane(points)
    X, Y, Z = get_plane_mesh(a, b, c, d, 
        (np.min(points[0]), np.max(points[0])),
        (np.min(points[1]), np.max(points[1]))
    )

    plt.figure(figsize=(12, 9), dpi=80)
    ax = plt.axes(projection='3d')
    ax.scatter3D(points[0], points[1], points[2], s=100)
    ax.scatter3D(centeroid[0], centeroid[1], centeroid[2], 'x', s=100)
    ax.plot_surface(X, Y, Z, alpha=0.5)
    plt.show()

def tri_point_method():
    A = np.array([3, 1, 1])
    B = np.array([1, 4, 2])
    C = np.array([1, 3, 4])
    # A = np.array([1, 0, 0])
    # B = np.array([0, 1, 0])
    # C = np.array([1, 0, 1])
    # A, B, C = C, B, A
    points = np.array([A, B, C])

    AB = B - A
    AC = C - A
    normal = np.cross(AB, AC)
    
    d = -C.dot(normal)
    a, b, c = normal

    print("%sx + %sy + %sz = %s" % (a, b, c, d))
    X, Y, Z = get_plane_mesh(a, b, c, d,
        (np.min(points[0]), np.max(points[0])),
        (np.min(points[1]), np.max(points[1]))
    )

    plt.figure(figsize=(12, 9), dpi=80)
    ax = plt.axes(projection='3d')
    ax.scatter3D(points[:, 0], points[:, 1], points[:, 2], s=100)
    ax.plot_surface(X, Y, Z, alpha=0.5)
    plt.show()


def load_points(path):
    with open(path) as file:
        contents = file.read()
    points = []
    for line in contents.splitlines():
        if line[0] != "\t":
            continue
        points.append(list(map(float, line[1:].split(", "))))
    return points

def test():
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np

    # These constants are to create random data for the sake of this example
    N_POINTS = 100
    TARGET_X_SLOPE = 10
    TARGET_y_SLOPE = 1
    TARGET_OFFSET  = 5
    EXTENTS = 5
    NOISE = 10

    # Create random data.
    # In your solution, you would provide your own xs, ys, and zs data.

    # xs = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
    # ys = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
    # zs = []
    # for i in range(N_POINTS):
    #     zs.append(xs[i]*TARGET_X_SLOPE + \
    #             ys[i]*TARGET_y_SLOPE + \
    #             TARGET_OFFSET + np.random.normal(scale=NOISE))
    
    # xs = np.array([3, 1, 1], dtype=np.float64)
    # ys = np.array([1, 4, 3], dtype=np.float64)
    # zs = np.array([1, 2, 4], dtype=np.float64)

    # xs = np.array([0, 1, 1], dtype=np.float64)
    # ys = np.array([1, 0, 0], dtype=np.float64)
    # zs = np.array([0, 0, 1], dtype=np.float64)

    points = np.array(load_points("data.txt"))
    xs = points[:, 0]
    ys = points[:, 1]
    zs = points[:, 2]

    xs -= np.mean(xs)
    ys -= np.mean(ys)
    zs -= np.mean(zs)

    # plot raw data
    plt.figure(figsize=(12, 9), dpi=80)
    ax = plt.subplot(111, projection='3d')
    ax.scatter(xs, ys, zs, color='b')

    # do fit
    tmp_A = []
    tmp_b = []
    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    # Manual solution
    # fit = (A.T * A).I * A.T * b
    A_inv = np.linalg.pinv(A)
    # print(A_inv.tolist())
    fit = A_inv * b
    # errors = b - A * fit
    # residual = np.linalg.norm(errors)

    # Or use Scipy
    # from scipy.linalg import lstsq
    # fit, residual, rnk, s = lstsq(A, b)

    print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    # print("errors: \n", errors)
    # print("residual:", residual)

    fit = np.array(fit.T)[0]
    a = fit[0]
    b = fit[1]
    c = -1.0
    d = fit[2]

    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X, Y, Z = get_plane_mesh(a, b, c, d, xlim, ylim)
    ax.plot_surface(X, Y, Z, alpha=0.5)
    limits = (np.min([X, Y, Z]), np.max([X, Y, Z]))
    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[0], limits[1])
    ax.set_zlim(limits[0], limits[1])

    point1 = np.array([1.0, 0.0, 1.0])
    point2 = np.array([0.0, 1.0, 1.0])
    vec1 = np.array([1.0, 0.0, np.dot(fit, point1)])
    vec2 = np.array([0.0, 1.0, np.dot(fit, point2)])
    cross = np.cross(vec1, vec2)
    origin = [0.0, 0.0, 0.0]

    print("normal: %s" % cross)
    print("quat: %s" % plane_to_quat(cross[0], cross[1], cross[2]))
    magnify = (limits[1] - limits[0]) / 5.0
    cross *= magnify
    vec1 *= magnify
    vec2 *= magnify

    # plot plane
    # xlim = ax.get_xlim()
    # ylim = ax.get_ylim()
    # X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
    #                 np.arange(ylim[0], ylim[1]))
    # Z = np.zeros(X.shape)
    # for r in range(X.shape[0]):
    #     for c in range(X.shape[1]):
    #         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    # ax.plot_wireframe(X,Y,Z, color='k')

    ax.quiver(origin[0], origin[1], origin[2], cross[0], cross[1], cross[2])
    ax.quiver(origin[0], origin[1], origin[2], vec1[0], vec1[1], vec1[2])
    ax.quiver(origin[0], origin[1], origin[2], vec2[0], vec2[1], vec2[2])

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

def main():
    # tri_point_method()
    # svd_method()
    test()

main()
