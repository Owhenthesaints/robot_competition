import numpy as np

data = np.loadtxt('imu_vals_10s_arr.csv', delimiter=',')

cov_mat = np.cov(data, rowvar=False)

print("Covariance Matrix :\n", cov_mat)

np.savetxt("imu_cov_matrix.csv", cov_mat, delimiter=',')
