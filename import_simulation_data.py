import bpy
import numpy as np
import os
import sys
import csv
from mathutils import Euler

if __name__ == "__main__":
    j = 0
    quad = bpy.data.objects['QuadFrame']
    quad.rotation_euler = Euler((0, 0, 0), 'ZYX')

    with open('src/output.csv', newline='') as csv_file:
        data = csv.reader(csv_file, delimiter=',')
        for params in data:
            quad.location = (float(params[0]), float(params[1]), float(params[2]))
            quad.convert_space(from_space='WORLD', to_space='LOCAL')
            quad.rotation_euler.rotate_axis("X", float(params[3]))
            quad.rotation_euler.rotate_axis("Y", float(params[4]))
            quad.rotation_euler.rotate_axis("Z", float(params[5]))
            quad.convert_space(from_space='LOCAL', to_space='WORLD')
            quad.keyframe_insert(data_path = "location", frame = j)
            quad.keyframe_insert(data_path = "rotation_euler", frame = j)
            j += 1