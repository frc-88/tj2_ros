# https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

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


def test():
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np

    # These constants are to create random data for the sake of this example
    N_POINTS = 100
    TARGET_X_SLOPE = 0
    TARGET_y_SLOPE = 0
    TARGET_OFFSET  = 5
    EXTENTS = 5
    NOISE = 0

    # Create random data.
    # In your solution, you would provide your own xs, ys, and zs data.
    xs = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
    ys = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
    zs = []
    for i in range(N_POINTS):
        zs.append(xs[i]*TARGET_X_SLOPE + \
                ys[i]*TARGET_y_SLOPE + \
                TARGET_OFFSET + np.random.normal(scale=NOISE))

    # plot raw data
    plt.figure()
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
    fit = (A.T * A).I * A.T * b
    errors = b - A * fit
    residual = np.linalg.norm(errors)

    # Or use Scipy
    # from scipy.linalg import lstsq
    # fit, residual, rnk, s = lstsq(A, b)

    print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    print("errors: \n", errors)
    print("residual:", residual)

    # plot plane
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                    np.arange(ylim[0], ylim[1]))
    Z = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    ax.plot_wireframe(X,Y,Z, color='k')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

def main():
    # tri_point_method()
    # svd_method()
    test()

main()
