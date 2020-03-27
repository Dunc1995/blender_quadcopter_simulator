import numpy as np
import constants

class pid():
    def __init__(self, kd = 1, kp = 6, ki = 0.1, samples_per_second =constants.TIME_STEPS_PER_SECOND):
        # total = state.m * state.g / state.k / ... 
        #     (cos(state.integral(1)) * cos(state.integral(2)))
        self.kd = kd
        self.kp = kp
        self.ki = ki
        self.sample_rate = samples_per_second

        self.angular_displacement_reading = np.array([0.0, 0, 0])
        self.angular_displacement_error_reading = np.array([0.0, 0, 0])
        

    def update_controller_state(self, thetadot):
        '''
        Updates the controller readings according to the controller's sample rate.
        Sample rate defaults to the simulation's time step which is a highly idealized default.
        '''
        err = self.kd * thetadot + self.kp * self.angular_displacement_reading + self.ki * self.angular_displacement_error_reading

        self.angular_displacement_reading += (thetadot/self.sample_rate)
        self.angular_displacement_error_reading += (self.angular_displacement_reading/self.sample_rate)
        return err