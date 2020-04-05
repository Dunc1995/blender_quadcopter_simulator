import numpy as np
import matplotlib.pyplot as plt
import constants
import physics_models

def run_simulation():
    rigid_body = physics_models.quadcopter()
    parameters = []
    steps = constants.SECONDS_OF_SIMULATION*constants.TIME_STEPS_PER_SECOND
    x_position_array = []
    y_position_array = []
    z_position_array = []

    with open('output.csv', 'w+') as f:
        f.write('x_position, y_position, z_position\n')
        for i in range(steps):
            rigid_body.update_state()

            if (i % constants.FRAME_RESOLUTION == 0): #if divisible by the Frame Resolution, then record the position.
                pos = rigid_body.get_position()
                print('Iteration: {} | {}'.format(i, pos))
                f.write('{}, {}, {}\n'.format(pos[0], pos[1], pos[2]))
                x_position_array.append(pos[0])
                y_position_array.append(pos[1])
                z_position_array.append(pos[2])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_position_array, y_position_array, z_position_array, marker='.')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.show()

if __name__ == "__main__":
    run_simulation()