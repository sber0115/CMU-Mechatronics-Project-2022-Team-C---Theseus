from black import out
import numpy as np
from sklearn.cluster import MeanShift

a = np.array([[1323, 678],
              [1356, 653],
              [1380, 680],
              [1290, 620],
              [230, 453],
              [223, 512]])
mean, stdev = np.mean(a, axis=0), np.std(a, axis=0)

outliers = ((np.abs(a[:,0] - mean[0]) > stdev[0])
            * (np.abs(a[:,1] - mean[1]) > stdev[1]))

inliers = [list(a[i]) for i in range(len(outliers)) if outliers[i] == 0]
mean_coor = np.around(np.mean(inliers, axis=0), 2)
print(outliers)
print(mean_coor)