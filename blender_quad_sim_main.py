import bpy
import numpy as np
from mathutils import Euler
from math import sin, cos

quad = bpy.data.objects['QuadFrame']

ANIMATION_FRAMES = 250
ANIMATION_FRAMES_PER_SECOND = 25
SECONDS_OF_SIMULATION = round(ANIMATION_FRAMES/ANIMATION_FRAMES_PER_SECOND) #Derived from Animation configuration for simplicity.
TIME_STEPS_PER_SECOND = 100 #Higher numbers will result in better simulation accuracy.
FRAME_RESOLUTION = round(TIME_STEPS_PER_SECOND/ANIMATION_FRAMES_PER_SECOND) #Used for mapping the simulation steps to animation frames.

class body():
    def __init__(self):
        self.mass = 1 #1kg
        self.arm_length = 0.6 #60cm
        self.k = 3e-6
        self.b = 1e-7
        self.inertia_tensor = np.diag(np.array([0.033, 0.033, 0.066]))
        self.positional_state = np.array([
            [0, 0, 0], #x
            [0, 0, 0], #y
            [1, 0, -0.98] #z
            ])
        #print(self.state)
    
    def calculate_integrals(self):
        t = TIME_STEPS_PER_SECOND
        for row in self.positional_state:
            acceleration = row[2]
            row[1] += (acceleration / t) #velocity Positionelement
            row[0] += (row[1] / t) #positional element
    
    def get_position(self):
        return self.positional_state[0:3, 0]

    def omega2thetadot(self, omega, angles):
        phi = angles[0]
        theta = angles[1]
        W = np.array([
            [1, 0, -sin(theta)],
            [0, cos(phi), cos(theta)*sin(phi)],
            [0, -sin(phi), cos(theta)*cos(phi)]
        ])
        return np.linalg.inv(W).dot(omega)
    
    def thetadot2omega(self, thetadot, angles):
        phi = angles[0]
        theta = angles[1]
        W = np.array([
            [1, 0, -sin(theta)],
            [0, cos(phi), cos(theta)*sin(phi)],
            [0, -sin(phi), cos(theta)*cos(phi)]
        ])
        return W.dot(thetadot)

    def angular_acceleration(self, inputs, omega):
        tau = self.torques(inputs)
        omega_dot = np.linalg.inv(self.inertia_tensor).dot((tau - np.cross(omega, self.inertia_tensor.dot(omega))))
        return omega_dot

    def torques(self, inputs):
        tau = np.array([
            self.arm_length * self.k * (inputs[0] - inputs[2]),
            self.arm_length * self.k * (inputs[1] - inputs[3]),
            self.b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
        ])
        return tau

    def get_thrust_from_controller_data(self, controller_data):
        e1 = controller_data[0]
        e2 = controller_data[1]
        e3 = controller_data[2]
        Ix = self.inertia_tensor[0][0]
        Iy = self.inertia_tensor[1][1]
        Iz = self.inertia_tensor[2][2]
        k = self.k
        L = self.arm_length
        b = self.b

        #TODO readd total/4 for countering gravity.
        inputs = np.array([0, 0, 0, 0])
        inputs[0] = -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[1] = e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        inputs[2] = -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[3] = e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)
        return inputs

class controller():
    def __init__(self, kd = 4, kp = 3, ki = 3, samples_per_second =TIME_STEPS_PER_SECOND):
        # total = state.m * state.g / state.k / ... 
        #     (cos(state.integral(1)) * cos(state.integral(2)))
        self.kd = kd
        self.kp = kp
        self.ki = ki
        self.sample_rate = samples_per_second
        self.angular_acceleration_reading = np.array([0.0, 0, 0])
        self.angular_velocity_reading = np.array([0.0, 0, 0])
        self.angular_displacement_reading = np.array([0.0, 0, 0])
        

    def update(self, thetadot):
        err = self.kd * thetadot + self.kp * self.angular_velocity_reading + self.ki * self.angular_displacement_reading
        print(err)

        self.angular_velocity_reading += (thetadot/self.sample_rate)
        self.angular_displacement_reading += (self.angular_velocity_reading/self.sample_rate)
        return err

if __name__ == "__main__":

    rigid_body = body()
    controller = controller()
    j=0
    theta = np.array([0, 0, 0.0])
    thetadot = np.array([0, 0.5, 0.0])

    steps = SECONDS_OF_SIMULATION*TIME_STEPS_PER_SECOND    
    for i in range(steps):
        rigid_body.calculate_integrals()
    
        omega = rigid_body.thetadot2omega(thetadot, theta)
        error = controller.update(thetadot)
        thrust = rigid_body.get_thrust_from_controller_data(error)
        omega_dot = rigid_body.angular_acceleration(thrust, omega)

        omega += (omega_dot/TIME_STEPS_PER_SECOND)
        thetadot = rigid_body.omega2thetadot(omega, theta)
        theta += (thetadot/TIME_STEPS_PER_SECOND)
        

        if (i % FRAME_RESOLUTION == 0): #if divisible by the Frame Resolution, then record the position.
            pos = rigid_body.get_position()
            print('Iteration: {} | {}'.format(i, pos))
            # print(theta)
            # print(omega_dot)
            quad.location = (pos[0],pos[1],pos[2])
            quad.rotation_euler = Euler((theta[0], theta[1], theta[2]), 'ZYX')
            quad.keyframe_insert(data_path = "location", frame = j)
            quad.keyframe_insert(data_path = "rotation_euler", frame = j)
            j += 1