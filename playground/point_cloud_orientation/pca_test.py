import numpy as np

from scipy import stats

from sklearn.decomposition import PCA

import matplotlib.pyplot as plt

# unused but required import for doing 3d projections with matplotlib < 3.2
import mpl_toolkits.mplot3d  # noqa: F401


def pdf(x, e):
    return 0.5 * (stats.norm(scale=0.25 / e).pdf(x) + stats.norm(scale=4 / e).pdf(x))


def create_data():
    e = np.exp(1)
    np.random.seed(4)

    y = np.random.normal(scale=1.0, size=(30000))
    x = np.random.normal(scale=1.0, size=(30000))
    z = np.random.normal(scale=1.0, size=len(x))

    density = pdf(x, e) * pdf(y, e)
    pdf_z = pdf(5 * z, e)

    density *= pdf_z

    a = x + y
    b = 2 * y
    c = a - b + z

    norm = np.sqrt(a.var() + b.var())
    a /= norm
    b /= norm

    return a, b, c, density


def plot_figs(a, b, c, density, fig_num, elev, azim):
    fig = plt.figure(fig_num, figsize=(4, 3))
    plt.clf()
    ax = fig.add_subplot(111, projection="3d", elev=elev, azim=azim)
    ax.set_position([0, 0, 0.95, 1])

    ax.scatter(a[::10], b[::10], c[::10], c=density[::10], marker="+", alpha=0.4)
    Y = np.c_[a, b, c]

    # Using SciPy's SVD, this would be:
    # _, pca_score, Vt = scipy.linalg.svd(Y, full_matrices=False)

    pca = PCA(n_components=3)
    pca.fit(Y)
    V = pca.components_.T

    x_pca_axis, y_pca_axis, z_pca_axis = 3 * V
    x_pca_plane = np.r_[x_pca_axis[:2], -x_pca_axis[1::-1]]
    y_pca_plane = np.r_[y_pca_axis[:2], -y_pca_axis[1::-1]]
    z_pca_plane = np.r_[z_pca_axis[:2], -z_pca_axis[1::-1]]
    x_pca_plane.shape = (2, 2)
    y_pca_plane.shape = (2, 2)
    z_pca_plane.shape = (2, 2)
    ax.plot_surface(x_pca_plane, y_pca_plane, z_pca_plane)
    # ax.xaxis.set_ticklabels([])
    # ax.yaxis.set_ticklabels([])
    # ax.zaxis.set_ticklabels([])

    limits = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()])
    # lower_limit = np.min(limits[:, 0])
    # upper_limit = np.max(limits[:, 1])
    limit = np.abs(np.max(limits))

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)


def main():
    a, b, c, density = create_data()
    elev = -40
    azim = -80
    plot_figs(a, b, c, density, 1, elev, azim)

    # elev = 30
    # azim = 20
    # plot_figs(a, b, c, density, 2, elev, azim)

    plt.show()


if __name__ == "__main__":
    main()

