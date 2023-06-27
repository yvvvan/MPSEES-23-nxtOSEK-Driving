"""
This file plots the given coordinates in a 2D plot.
The coordinates are saved in a .txt file.
"""
import matplotlib.pyplot as plt

coords_file = open("coordinates.txt", "r")

lines = coords_file.readlines()
x = [lines.split(", ")[0] for lines in lines]
y = [lines.split(", ")[1] for lines in lines]

coords_file.close()

plt.plot(x, y)
plt.show()