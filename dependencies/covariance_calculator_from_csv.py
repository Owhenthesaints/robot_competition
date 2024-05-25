import numpy as np

data = np.loadtxt('cov_arr.csv', delimiter=',')

cov_mat = np.cov(data, rowvar=False)

print("Covariance Matrix :\n", cov_mat)
