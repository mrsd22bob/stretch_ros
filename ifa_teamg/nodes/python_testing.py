#! python
#  == METHOD 2 ==
from scipy import optimize
import numpy as np


x = np.asarray([0, 1, 0]).reshape((1,-1))

y = np.asarray([-1, 0, 1]).reshape((1,-1))

z = np.asarray([0, 0, 0]).reshape((1,-1))

cluster = np.vstack((x,y,z))

print(cluster[0,:])

print(cluster.shape)

method_2 = "leastsq"
def find_centroids(cluster):

    x = cluster[0,:]
    y = cluster[1,:]

    x_m = np.average(x)
    y_m = np.average(y)

    center_estimate = x_m, y_m
    center_2, ier = optimize.leastsq(f_2, center_estimate, args=(cluster))

    xc_2, yc_2 = center_2
    Ri_2       = calc_R(*center_2, cluster)
    R_2        = Ri_2.mean()
    residu_2   = sum((Ri_2 - R_2)**2)

    print(xc_2, yc_2)

    return xc_2, yc_2


def calc_R(xc, yc, cluster):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    x = cluster[0,:]
    y = cluster[1,:]
    return np.sqrt((x-xc)**2 + (y-yc)**2)


def f_2(c, cluster):
    """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c, cluster)
    print(Ri - Ri.mean())
    return Ri - Ri.mean()

center_x, center_y = find_centroids(cluster)
# print(center_x, center_y)