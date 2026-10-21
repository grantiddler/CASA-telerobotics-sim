import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Generate random data
x = np.random.rand(100)
y = np.random.rand(100)
z = np.random.rand(100)

# Create a new figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create a scatter plot
# ax.scatter(x, y, z, c='blue', marker='o')

# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

x = []
y = []
z = []
err = []

with open('log.txt', 'r') as file:
    for line in file:
        if line.strip()[0:12] == "[optimize-2]":
            A = line.strip().split('|')
            print(A)

            x.append(float(A[3][6:-6]))
            y.append(float(A[4][6:-6]))
            z.append(float(A[5][6:-6]))
            err.append(float(A[2][6:-6]))

p = ax.scatter(x, y, z, c= err,cmap = "magma")
fig.colorbar(p, ax=ax)
plt.show()