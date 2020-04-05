import numpy as np
from math import sin, cos, pi
import constants
import physics_models

def run_simulation():
    rigid_body = physics_models.quadcopter()
    parameters = []

    steps = constants.SECONDS_OF_SIMULATION*constants.TIME_STEPS_PER_SECOND    
    for i in range(steps):
        rigid_body.update_state()

        if (i % constants.FRAME_RESOLUTION == 0): #if divisible by the Frame Resolution, then record the position.
            pos = rigid_body.get_position()
            print('Iteration: {} | {}'.format(i, pos))
            parameters.append(np.array([pos, rigid_body.euler_angle_time_derivatives]))
    return parameters
            

if __name__ == "__main__":
    run_simulation()