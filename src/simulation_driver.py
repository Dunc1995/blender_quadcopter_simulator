#import bpy
import numpy as np
#from mathutils import Euler
from math import sin, cos, pi
import constants
import physics_models

#quad = bpy.data.objects['QuadFrame']

def main():
    rigid_body = physics_models.quadcopter()
    #quad.rotation_euler = Euler((theta[0], theta[1], theta[2]), 'ZYX')
    
    j=0
    steps = constants.SECONDS_OF_SIMULATION*constants.TIME_STEPS_PER_SECOND    
    for i in range(steps):
        rigid_body.update_state()

        if (i % constants.FRAME_RESOLUTION == 0): #if divisible by the Frame Resolution, then record the position.
            pos = rigid_body.get_position()
            print('Iteration: {} | {}'.format(i, pos))
            # print(theta)
            # print(theta)
            # print(omega_dot)
            # quad.location = (pos[0],pos[1],pos[2])
            # quad.convert_space(from_space='WORLD', to_space='LOCAL')
            # quad.rotation_euler.rotate_axis("X", thetadot[0])
            # quad.rotation_euler.rotate_axis("Y", thetadot[1])
            # quad.rotation_euler.rotate_axis("Z", thetadot[2])
            # quad.convert_space(from_space='LOCAL', to_space='WORLD')
            # quad.keyframe_insert(data_path = "location", frame = j)
            # quad.keyframe_insert(data_path = "rotation_euler", frame = j)
            j += 1

if __name__ == "__main__":
    main()