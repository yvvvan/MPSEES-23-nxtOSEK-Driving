"""
This file plots the given coordinates in a 2D plot.
The coordinates are saved in a .txt file.
"""
import matplotlib.pyplot as plt
import numpy as np

coords_file = open("../testfiles/example_logs/coords.txt", "r")

lines = coords_file.readlines()
x = np.array([float(line.split(" ")[0]) for line in lines])
y = np.array([float(line.split(" ")[1]) for line in lines])

coords_file.close()

max = np.max(x)
min = np.min(x)
steps = (max - min) / 10

fig, axs = plt.subplots(3, 1)

axs[0].plot(x, y)
axs[0].set_xticks(np.arange(np.min(x), np.max(x), steps))
axs[0].set_yticks(np.arange(np.min(y), np.max(y), steps))

shift_file = open("../testfiles/example_logs/shift.txt", "r")
lines = shift_file.readlines()
shift_file.close()
x = np.array([float(line.split(" ")[0]) for line in lines])
y = np.array([float(line.split(" ")[1]) for line in lines])
time = np.array([float(line.split(" ")[2]) for line in lines])

# axs[1].plot(time, x)
axs[1].plot(time, y)
axs[1].legend(["x", "y"])

angle_file = open("../testfiles/example_logs/angles.txt", "r")
lines = angle_file.readlines()
angle_file.close()
angle = np.array([float(line.split(" ")[0]) for line in lines])
speed = np.array([float(line.split(" ")[1]) for line in lines])
angle_time = np.array([float(line.split(" ")[2]) for line in lines])
intersection = np.array([float(line.split(" ")[3]) for line in lines])

axs[2].plot(angle_time, angle)
# axs[2].plot(angle_time, speed * 100)
axs[2].plot(angle_time, intersection*10)
axs[2].legend(["Angle", "Speed"])
plt.tight_layout()
plt.show()