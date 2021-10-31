import odrive
from odrive.enums import *
from odrive.utils import *
import time


class odriveControl():

    def __init__(self):
        self.odrive_inst = odrive.find_any()

    def calibration_sequence(self):
        print("AXIS_STATE_MOTOR_CALIBRATION")
        self.odrive_inst.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(10)
        self.odrive_inst.axis0.motor.config.pre_calibrated = True
        print("AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION")
        self.odrive_inst.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        time.sleep(10)
        print("AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
        self.odrive_inst.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(10)
        self.odrive_inst.axis0.encoder.config.pre_calibrated = True
        self.odrive_inst.save_configuration()
        print("Saving")
        time.sleep(2)

    def torque(self, t, dur):
        self.odrive_inst.axis0.controller.enable_current_mode_vel_limit = False
        self.odrive_inst.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.odrive_inst.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrive_inst.axis0.controller.input_torque = t
        time.sleep(dur)
        self.odrive_inst.axis0.controller.input_torque = 0
        self.odrive_inst.axis0.requested_state = AXIS_STATE_IDLE
        time.sleep(1)

    def tune_current_control_bandwidth(self):
        for i in range(1500, 3000, 100):
            print(i)
            self.odrive_inst.axis0.motor.config.current_control_bandwidth = i
            self.torque(-0.1, 1)
            self.torque(0.1, 1)

    def print_tunable_params(self):
        print("Position Gain: ", self.odrive_inst.axis0.controller.config.pos_gain)
        print("Velocity Gain: ", self.odrive_inst.axis0.controller.config.vel_gain)
        print("Velocity Integrator Gain: ", self.odrive_inst.axis0.controller.config.vel_integrator_gain)
        # print("Bandwidth: ", self.odrive_inst.axis0.controller.config.bandwidth)
        print("Current Control Bandwidth: ", self.odrive_inst.axis0.motor.config.current_control_bandwidth)


