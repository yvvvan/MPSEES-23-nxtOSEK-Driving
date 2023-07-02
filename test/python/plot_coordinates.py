"""
This file plots the given coordinates in a 2D plot.
The coordinates are saved in a .txt file.
"""
import matplotlib.pyplot as plt
import numpy as np

coords_file = open("../testfiles/example_logs/coords.txt", "r")

lines = coords_file.readlines()
x = np.array([float(lines.split(" ")[0]) for lines in lines])
y = np.array([float(lines.split(" ")[1]) for lines in lines])

coords_file.close()

max = np.max(x)
min = np.min(x)
steps = (max - min) / 10

plt.plot(x, y)
plt.xticks(np.arange(np.min(x), np.max(x), steps))
plt.yticks(np.arange(np.min(y), np.max(y), steps))
plt.show()