import argparse
import os
from itertools import product

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sklearn.cluster import KMeans
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel
from sklearn.neighbors import LocalOutlierFactor


def main(data_dir):
    # Hyperparameters
    N_clusters = 50
    outlier_ratio = 0.3
    size_lookup_table = 50
    to_single_rotor_map = False

    # load data
    print("Loading data from directory %s" % data_dir)
    list_subfolders_with_paths = [f.path for f in os.scandir(data_dir) if f.is_dir()]

    for idx, directory in enumerate(list_subfolders_with_paths):
        if idx == 0:
            cmd_data = np.loadtxt(os.path.join(directory, "cmd_data.csv"), delimiter=',')[:, np.newaxis]
            force_data = np.loadtxt(os.path.join(directory, "force_data.csv"), delimiter=',')
            torque_data = np.loadtxt(os.path.join(directory, "torque_data.csv"), delimiter=',')
            voltage_data = np.loadtxt(os.path.join(directory, "voltage_data.csv"), delimiter=',')[:, np.newaxis]
        else:
            cmd_data = np.concatenate(
                [cmd_data, np.loadtxt(os.path.join(directory, "cmd_data.csv"), delimiter=',')[:, np.newaxis]])
            force_data = np.concatenate(
                [force_data, np.loadtxt(os.path.join(directory, "force_data.csv"), delimiter=',')])
            torque_data = np.concatenate(
                [torque_data, np.loadtxt(os.path.join(directory, "torque_data.csv"), delimiter=',')])
            voltage_data = np.concatenate(
                [voltage_data, np.loadtxt(os.path.join(directory, "voltage_data.csv"), delimiter=',')[:, np.newaxis]])

    
    # measured 4 rotors but if require single rotor map, divide by 4
    if to_single_rotor_map:
        force_data /= 4.0

    print("Performing outlier removal...")
    X = np.concatenate([cmd_data, voltage_data, force_data[:, 2][:, np.newaxis]], axis=1)
    clf = LocalOutlierFactor(n_neighbors=20, contamination='auto')
    y_pred = clf.fit_predict(X)
    X_scores = clf.negative_outlier_factor_
    # sort the scores and select the top k-percent quantile
    X_scores_sorted = np.sort(X_scores)
    cutoff_score = X_scores_sorted[int(outlier_ratio * X_scores_sorted.shape[0])]
    X_select = X_scores > cutoff_score
    X_filtered = X[X_select]

    print("Clustering data...")
    kmeans = KMeans(n_clusters=N_clusters, random_state=0).fit(X_filtered)
    X_clustered = kmeans.cluster_centers_

    print("Fitting data...")
    X_gp = X_clustered[:, 1:]
    y_gp = X_clustered[:, 0]

    kernel = ConstantKernel(1.0, (1e-3, 1e3)) * RBF([1, 1], (1.0e-5, 1.0e5)) \
             + WhiteKernel(noise_level=1e-2,
                           noise_level_bounds=(1e-2, 1e+1))
    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=15)
    gp.fit(X_gp, y_gp)

    # Input space
    x1 = np.linspace(X_clustered[:, 1].min(), X_clustered[:, 1].max(), size_lookup_table)  # p
    x2 = np.linspace(X_clustered[:, 2].min(), X_clustered[:, 2].max(), size_lookup_table)  # q
    x1x2 = np.array(list(product(x1, x2)))
    y_pred, MSE = gp.predict(x1x2, return_std=True)

    X0p = x1x2[:, 0].reshape(size_lookup_table, size_lookup_table)
    X1p = x1x2[:, 1].reshape(size_lookup_table, size_lookup_table)
    Zp = np.reshape(y_pred, (size_lookup_table, size_lookup_table))

    print("Plotting results...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X0p, X1p, Zp, rstride=1, cstride=1, cmap='gist_rainbow', linewidth=0, antialiased=True)
    ax.scatter(voltage_data, force_data[:, 2], cmd_data, color='b', s=1., label='Data points')
    ax.scatter(X_filtered[:, 1], X_filtered[:, 2], X_filtered[:, 0], color='k', s=10., label='Filtered data points')
    ax.scatter(X_clustered[:, 1], X_clustered[:, 2], X_clustered[:, 0], color='r', s=50., label='Clustered data points')
    ax.set_xlabel('Voltage [V]')
    ax.set_ylabel('Force [N]')
    ax.set_zlabel('Command [-]')
    plt.show()

    fig, ax = plt.subplots(1, 1)
    step = 50
    m = np.amax(Zp)
    levels = np.arange(0.0, m, step) + step
    cp = ax.contourf(X0p, X1p, Zp, levels, cmap='jet', alpha=0.8)
    fig.colorbar(cp)
    ax.scatter(X_filtered[:, 1], X_filtered[:, 2],
               color='k', s=1., label='Filtered data points')
    ax.set_title('Command Contour Plot')
    ax.set_xlabel('Voltage [V]')
    ax.set_ylabel('Force [N]')
    plt.show()

    lookup_table_fname = os.path.join(data_dir, 'thrust_map.csv')
    print(
        "Saving lookup table of size [%dx%d] to [%s] ..." % (size_lookup_table, size_lookup_table, lookup_table_fname))
    print("Rows denote voltage in the range [%.2f, %.2f] V in increasing order." % (np.min(X0p), np.max(X0p)))
    print("Columns denote force in the range [%.2f, %.2f] N in increasing order." % (np.min(X1p), np.max(X1p)))

    # we add some metainformation at the beginning of the thrust map
    meta_info = np.zeros((1, size_lookup_table))
    meta_info[0, 0] = size_lookup_table
    meta_info[0, 1] = np.min(X1p)
    meta_info[0, 2] = np.max(X1p)
    meta_info[0, 3] = np.min(X0p)
    meta_info[0, 4] = np.max(X0p)
    Zp = np.concatenate([meta_info, Zp], axis=0)
    np.savetxt(lookup_table_fname, Zp, delimiter=',', fmt='%f')

    print("Done!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Identify thrust map.')
    parser.add_argument('--data_dir',
                        help='Directory of thrust data', required=True)
    args = parser.parse_args()

    data_dir = args.data_dir
    main(data_dir=data_dir)
