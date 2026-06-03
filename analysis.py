import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

width = 0.21779 * 2
radius = .05



def average_slip(filename):

    slip_trans = []
    slip_tan = []
    slip_rot = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)  # Reader object

        fields = next(csvreader)  # Read header
        for row in csvreader:     # Read rows

            r = R.from_quat(row[-15:-11])

            heading = r.as_euler('xyz')[0]

            v_ideal = radius * (float(row[-11]) + float(row[-10]) + float(row[-9]) + float(row[-8])) / 4
            v_tan = np.cos(heading) * float(row[-7]) - np.sin(heading) * float(row[-6])
            v_trans =  np.sin(heading) * float(row[-7]) + np.cos(heading) * float(row[-6])


            omega_ideal = radius * (float(row[-11]) + float(row[-10]) - float(row[-9]) - float(row[-8])) / (2 * width)
            omega_real = float(row[-2])

            rotational_slip = omega_real - omega_ideal
            tangential_slip = v_tan - v_ideal
            transverse_slip = v_trans

            slip_trans.append(transverse_slip)
            slip_tan.append(tangential_slip)
            slip_rot.append(rotational_slip)

    return [np.average(slip_tan), np.average(slip_trans),np.average(slip_rot)]

r = []
trans = []
tan = []
for i in fnames:
    temp =average_slip(i)
    tan.append(temp[0])
    trans.append(temp[1])
    r.append(temp[2])

    print(temp)
            
steps = [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]

plt.subplot(1, 3, 1)
plt.plot([-5, -4, -3, -2, -1, 0, 1, 3, 5], r[0:9])
plt.title("Rotational slip")
plt.xlabel("")

plt.subplot(1, 3, 2)
plt.plot([-5, -4, -3, -2, -1, 0, 1, 3, 5],tan[0:9])
plt.title("Tangential slip")


plt.subplot(1, 3, 3)
plt.plot([-5, -4, -3, -2, -1, 0, 1, 3, 5], trans[0:9])
plt.title("Transverse slip")


plt.subplot(1, 3, 1)
plt.plot([-5, -4, -3, -2, 0, 1, 2, 3, 4, 5], r[9:19])

plt.subplot(1, 3, 2)
plt.plot([-5, -4, -3, -2, 0, 1, 2, 3, 4, 5], tan[9:19])

plt.subplot(1, 3, 3)
plt.plot([-5, -4, -3, -2, 0, 1, 2, 3, 4, 5], trans[9:19])

plt.subplot(1, 3, 1)
plt.plot([-5, -3, -1, 0, 1, 2, 3, 4, 5], r[20:])

plt.subplot(1, 3, 2)
plt.plot([-5, -3, -1, 0, 1, 2, 3, 4, 5], tan[20:])

plt.subplot(1, 3, 3)
plt.plot([-5, -3, -1, 0, 1, 2, 3, 4, 5], trans[20:])


plt.show()
