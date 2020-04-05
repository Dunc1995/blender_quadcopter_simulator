import bpy
import numpy as np
import os
import sys
from mathutils import Euler

if __name__ == "__main__":
    j = 0
    # quad = bpy.data.objects['QuadFrame']
    # quad.rotation_euler = Euler((0, 0, 0), 'ZYX')
    parameters = driver.get_simulation_results()

    for param in parameters:
        print(param)
        quad.location = (param[0][0],param[0][1],param[0][2])
        quad.convert_space(from_space='WORLD', to_space='LOCAL')
        quad.rotation_euler.rotate_axis("X", param[1][0])
        quad.rotation_euler.rotate_axis("Y", param[1][1])
        quad.rotation_euler.rotate_axis("Z", param[1][2])
        quad.convert_space(from_space='LOCAL', to_space='WORLD')
        quad.keyframe_insert(data_path = "location", frame = j)
        quad.keyframe_insert(data_path = "rotation_euler", frame = j)
        j += 1