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
    # x_ang_vel = []
    # y_ang_vel = []
    # z_ang_vel = []
    x_eul_vel = []
    y_eul_vel = []
    z_eul_vel = []

    with open('output.csv', 'w+') as f:
        for i in range(steps):
            rigid_body.update_state()

            if (i % constants.FRAME_RESOLUTION == 0): #if divisible by the Frame Resolution, then record the position.
                pos = rigid_body.get_position()
                print('Iteration: {} | {}'.format(i, pos))
                f.write('{},{},{},{},{},{}\n'.format(pos[0], pos[1], pos[2],
                rigid_body.euler_angle_time_derivatives[0],
                rigid_body.euler_angle_time_derivatives[1],
                rigid_body.euler_angle_time_derivatives[2]))
                x_position_array.append(pos[0])
                y_position_array.append(pos[1])
                z_position_array.append(pos[2])
                # x_ang_vel.append(rigid_body.angular_velocity[0])
                # y_ang_vel.append(rigid_body.angular_velocity[1])
                # z_ang_vel.append(rigid_body.angular_velocity[2])
                # x_eul_vel.append(rigid_body.euler_angles[0])
                # y_eul_vel.append(rigid_body.euler_angles[1])
                # z_eul_vel.append(rigid_body.euler_angles[2])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_position_array, y_position_array, z_position_array, marker='.')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    # plt.plot(x_ang_vel)
    # plt.plot(y_ang_vel)
    # plt.plot(z_ang_vel)
    # plt.grid(color='b', linestyle='-', linewidth=0.1)
    # plt.plot(x_eul_vel)
    # plt.plot(y_eul_vel)
    # plt.plot(z_eul_vel)
    plt.show()

if __name__ == "__main__":
    run_simulation()