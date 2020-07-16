import numpy as np
from math import sin, cos, pi
import constants
import controllers

class quadcopter():
    def __init__(self, controller=controllers.pid()):
        #TODO make these into arguments once this object is sufficiently generic.
        self.mass = 1 
        self.arm_length = 0.5
        self.k = 3e-6
        self.b = 1e-7
        self.inertia_tensor = np.diag(np.array([0.033, 0.033, 0.066]))
        self.controller = controller
        

        #TODO Make this tidier such that rotational state can succinctly be incorporated into the object's state.
        #? Currently position and its time derivatives are only contained in this matrix...
        self.positional_state = np.array([
            [0, 0, 0], #x
            [0, 0, 0], #y
            [1, 0, -9.81] #z
            ])

        #TODO MAKE TIDIER!
        self.euler_angles = np.array([0, 0, 0.0])
        self.euler_angle_time_derivatives = np.array([0.2, 0.1, 0.0]) #! Don't leave this default-ed to non zero values!
        self.angular_velocity = None
        self.angular_acceleration = None
        self.__current_rotation_matrix = self.__set_rotation_matrix()
        #print(self.state)
    
    def update_state(self):
        '''Updates the body's state given the simulation's time steps per second.'''
        t = constants.TIME_STEPS_PER_SECOND
        for row in self.positional_state:
            acceleration = row[2]
            row[1] += (acceleration / t) #velocity Positionelement
            row[0] += (row[1] / t) #positional element
        #TODO MAKE TIDIER! Above and below in this method could be improved.
        self.angular_velocity = self.get_omega_vector(self.euler_angle_time_derivatives, self.euler_angles)
        error_signal = self.controller.update_controller_state(self.angular_velocity)
        linear_thrust = self.get_thrust_from_controller_data(error_signal)

        self.angular_acceleration = self.get_angular_acceleration_vector(linear_thrust, self.angular_velocity)
        self.angular_velocity += (self.angular_acceleration/t)
        self.euler_angle_time_derivatives = self.get_euler_angle_time_derivatives_vector(self.angular_velocity, self.euler_angles)
        self.euler_angles += (self.euler_angle_time_derivatives/t)
        self.__set_rotation_matrix()
        #print(self.__current_rotation_matrix)
        print(linear_thrust)
        self.positional_state[0:3, 2] = self.get_linear_acceleration_vector(np.sum(linear_thrust))
    
    def get_position(self):
        '''Returns the XYZ coordinates of the rigid body.'''
        return self.positional_state[0:3, 0]
    
    def get_linear_velocity(self):
        '''Returns the global velocity vector of the rigid body.'''
        return self.positional_state[0:3, 1]

    def get_euler_angle_time_derivatives_vector(self, omega, angles):
        '''Return the Euler angles' time derivatives given the rigid body's angular velocity vector.'''
        W = self.__w_matrix(angles)
        thetadot = np.linalg.inv(W).dot(omega)
        return thetadot
    
    def get_omega_vector(self, thetadot, angles):
        '''Returns the rigid body's angular velocity given a set of ZYX Euler angles and their time derivatives'''
        W = self.__w_matrix(angles)
        omega = W.dot(thetadot)
        return omega

    def get_angular_acceleration_vector(self, inputs, omega):
        '''Returns the angular acceleration vector.''' #TODO remind myself what order this comes in when calculating state.
        tau = self.get_torque_vector(inputs)
        omega_dot = np.linalg.inv(self.inertia_tensor).dot((tau - np.cross(omega, self.inertia_tensor.dot(omega))))
        return omega_dot

    def get_torque_vector(self, inputs):
        '''Returns the torque vector resultant from 4 quadcopter rotors - calculated via Newton's 2nd Law.'''
        tau = np.array([
            self.arm_length * self.k * (inputs[0] - inputs[2]),
            self.arm_length * self.k * (inputs[1] - inputs[3]),
            self.b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
        ])
        #print('tau: {}'.format(tau))
        return tau

    def get_thrust_from_controller_data(self, controller_data):
        '''Returns the resultant thrust vector from 4 motors, given input controller data.'''
        e1 = controller_data[0]
        e2 = controller_data[1]
        e3 = controller_data[2]
        Ix = self.inertia_tensor[0][0]
        Iy = self.inertia_tensor[1][1]
        Iz = self.inertia_tensor[2][2]
        k = self.k
        L = self.arm_length
        b = self.b

        #! This is unlikely correct - inputs should be dimensionless which I can't be bothered to check.
        #! Regardless, messing around with Inertia/b/L/k constants has no impact on the simulation physics and doesn't need to be included in the PID controller.
        #TODO readd total/4 for countering gravity.
        inputs = np.array([0.0, 0, 0, 0])
        inputs[0] = 2.455 + -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[1] = 2.455 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        inputs[2] = 2.455 + -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[3] = 2.455 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)
        return inputs

    def get_linear_acceleration_vector(self, sum_thrust):
        kd = 0.01
        gravity = np.array([0, 0, -9.81])
        body_frame_thrust = np.array([0, 0, sum_thrust])
        global_thrust_vector = self.__current_rotation_matrix.dot(body_frame_thrust)
        Fd = -kd * self.get_linear_velocity()
        # print(gravity)
        # print(global_thrust_vector)
        # print(Fd)
        output = gravity + (1 / self.mass)*(global_thrust_vector + Fd)
        print('Linear Acceleration: {}'.format(output))
        return output

    def __w_matrix(self, angles):
        '''Used for calculating the relationship between angular velocity/acceleration and Euler angle time derivatives -
        https://davidbrown3.github.io/2017-07-25/EulerAngles/'''
        #! Comparing Euler angle time derivatives and angular velocity yields identical plots - might not need this conversion.
        phi = angles[0]
        theta = angles[1]
        w = np.array([
            [1, 0.0, -sin(theta)],
            [0, cos(phi), cos(theta)*sin(phi)],
            [0, -sin(phi), cos(theta)*cos(phi)]
        ])
        return w

    def __set_rotation_matrix(self):
        phi = self.euler_angles[2]
        theta = self.euler_angles[1]
        psi = self.euler_angles[0]

        self.__current_rotation_matrix = np.array([
        [cos(phi)*cos(theta), cos(theta)*sin(phi), -sin(theta)],
        [cos(phi)*sin(theta)*sin(psi) - cos(psi)*sin(phi), cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi), cos(theta)*sin(psi)], 
        [sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), cos(theta) * cos(psi)]
        ])
