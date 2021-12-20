"""
R80 KV110 Motor Control for ODrive

@author: Jason
@date: 11/21/2021
"""

import sys
import time
import odrive
import math
from odrive.enums import *
from odrive.utils import start_liveplotter
#from odrive.pyfibre.fibre.protocol import ChannelBrokenException

class R80MotorControl:
    """
    Class for configuring an Odrive axis for R80 motor. 
    Only works with one ODrive at a time.

    """
    
    # Kv
    R80_KV = 110.0
        
    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 87e-6
    
    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 125e-3
    
    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 10 # TODO: not too sure

    def __init__(self, axis_num=0):
        """
        Initalizes R80MotorControl class by finding odrive
        and grabbing specified axis object.
        
        :param axis_num: Which channel/motor on the odrive your referring to.
        :type axis_num: int (0 or 1)
        """
        
        self.axis_num = axis_num
    
        # Connect to Odrive
        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")
        
    def _find_odrive(self):
        # connect to Odrive
        self.odrv = odrive.find_any()
        self.odrive_axis = getattr(self.odrv, "axis{}".format(self.axis_num))
        
    def configure(self):
        """
        Configures the ODrive device for R80 motor.
        """
        
        # Erase pre-exsisting configuration
        print("Erasing pre-exsisting configuration...")
        try:
            self.odrv.erase_configuration()
        except Exception:
            pass
        
        self._find_odrive()
        
        self.odrive_axis.motor.config.pole_pairs = 21

        self.odrive_axis.motor.config.resistance_calib_max_voltage = 5
        self.odrive_axis.motor.config.requested_current_range      = 25

        # Estimated KV but should be measured using the "drill test", which can
        # be found here:
        # https://discourse.odriverobotics.com/t/project-hoverarm/441
        self.odrive_axis.motor.config.torque_constant = 8.27 / self.R80_KV

        # motors contain hall effect sensors instead of incremental 
        # encorders
        self.odrive_axis.encoder.config.mode = ENCODER_MODE_HALL

        # The hall feedback has 6 states for every pole pair in the motor. Since
        # we have 21 pole pairs, we set the cpr to 21*6 = 126.
        self.odrive_axis.encoder.config.cpr = 126
        # Since hall sensors are low resolution feedback, we also bump up the 
        #offset calibration displacement to get better calibration accuracy.
        self.odrive_axis.encoder.config.calib_scan_distance = 150
        
        # Since the hall feedback only has 90 counts per revolution, we want to 
        # reduce the velocity tracking bandwidth to get smoother velocity 
        # estimates. We can also set these fairly modest gains that will be a
        # bit sloppy but shouldn’t shake your rig apart if it’s built poorly. 
        # Make sure to tune the gains up when you have everything else working 
        # to a stiffness that is applicable to your application.
        r80_motor.set_tunable_params(45, 0.006, 0.01, 300, 1500)
        self.odrive_axis.controller.config.vel_limit = 10

        # Set in position control mode so we can control the position of the 
        # wheel
        self.odrive_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # In the next step we are going to start powering the motor and so we 
        # want to make sure that some of the above settings that require a 
        # reboot are applied first.
        print("Saving manual configuration and rebooting...")
        self.odrv.save_configuration()
        print("Manual configuration saved.")
        try:
            self.odrv.reboot()
        except Exception:
            pass
            
        self._find_odrive()

        input("Make sure the motor is safe to move, then press enter...")
        
        print("Calibrating Odrive for motor (you should hear a "
        "beep)...")
        
        self.odrive_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration to take place
        time.sleep(10)

        if self.odrive_axis.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.odrive_axis.motor.error, 
                                self.odrive_axis.motor))
            
            sys.exit(1)

        if self.odrive_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.odrive_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.odrive_axis.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.odrive_axis.motor))
            
            sys.exit(1)

        if self.odrive_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.odrive_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.odrive_axis.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.odrive_axis.motor))
            
            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration 
        # to persistent memory is OK
        self.odrive_axis.motor.config.pre_calibrated = True

        # Check the alignment between the motor and the hall sensor. Because of 
        # this step you are allowed to plug the motor phases in random order and
        # also the hall signals can be random. Just don’t change it after 
        # calibration.
        print("Calibrating Odrive for encoder...")
        self.odrive_axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        time.sleep(10)
        self.odrive_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        # Wait for calibration to take place
        time.sleep(30)
            
        if self.odrive_axis.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.odrive_axis.encoder.error, 
                                         self.odrive_axis.encoder))
            
            sys.exit(1)
        
        # If offset_float isn't 0.5 within some tolerance, or its not 1.5 within
        # some tolerance, raise an error
        if not ((self.odrive_axis.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.odrive_axis.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.odrive_axis.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.odrive_axis.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.odrive_axis.encoder.config.offset_float, 
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                        self.odrive_axis.encoder))
                       
            sys.exit(1)
        
        # If all looks good, then lets tell ODrive that saving this calibration 
        # to persistent memory is OK
        self.odrive_axis.encoder.config.pre_calibrated = True
        
        print("Saving calibration configuration and rebooting...")
        self.odrv.save_configuration()
        print("Calibration configuration saved.")
        try:
            self.odrv.reboot()
        except Exception:
            pass
            
        self._find_odrive()
        
        print("Odrive configuration finished.")
    
    def mode_idle(self):
        """
        Puts the motor in idle (i.e. can move freely).
        """
        
        self.odrive_axis.requested_state = AXIS_STATE_IDLE
    
    def mode_close_loop_control(self):
        """
        Puts the motor in closed loop control.
        """
        
        self.odrive_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    def move_input_pos(self, angle):
        """
        Puts the motor at a certain angle.
        
        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """
        
        self.odrive_axis.controller.input_pos = angle/360.0
        
    def move_input_vel(self, velocity):
        """
        Puts the motor at a certain velocity.
        
        :param velocity: Velocity you want the motor to move.
        :type velocity: int or float
        """
        
        self.odrive_axis.controller.input_vel = velocity/360.0
        
    def move_input_torque(self, torque):
        """
        Puts the motor at a certain torque.
        
        :param torque: Torque you want the motor to move.
        :type torque: int or float
        """
        
        self.odrive_axis.controller.input_torque = torque
        
    def print_tunable_params(self):
        print("Position Gain: ", self.odrive_axis.controller.config.pos_gain)
        print("Velocity Gain: ", self.odrive_axis.controller.config.vel_gain)
        print("Velocity Integrator Gain: ", self.odrive_axis.controller.config.vel_integrator_gain)
        print("Bandwidth: ", self.odrive_axis.encoder.config.bandwidth)
        print("Current Control Bandwidth: ", self.odrive_axis.motor.config.current_control_bandwidth)
        
    def tune_current_control_bandwidth(self):
        self.odrive_axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        
        for i in range(1500, 3000, 100):
            print("Current Bandwidth: {}".format(i))
            self.odrive_axis.motor.config.current_control_bandwidth = i
            self.move_input_torque(-0.1)
            time.sleep(1)
            self.move_input_torque(0.1)
            time.sleep(1)
        
    def set_tunable_params(self, kp, kv, ki, 
                           bandwidth, 
                           current_control_bandwidth):
        """
        Configures motor tuning parameters.
        
        :param kp: Proportional gain
        :param kv: Velocity gain
        :param ki: Integral gain
        :param bandwidth: Bandwidth of the encoder
        :param current_control_bandwidth: Bandwidth of the current control
        """
        print("Setting params..")
        self.odrive_axis.controller.config.pos_gain = kp
        self.odrive_axis.controller.config.vel_gain = (kv * 
                                                       self.odrive_axis.motor.config.torque_constant * 
                                                       self.odrive_axis.encoder.config.cpr)
        self.odrive_axis.controller.config.vel_integrator_gain = (ki * 
                                                                  self.odrive_axis.motor.config.torque_constant * 
                                                                  self.odrive_axis.encoder.config.cpr)
        self.odrive_axis.encoder.config.bandwidth = bandwidth
        self.odrive_axis.motor.config.current_control_bandwidth = current_control_bandwidth
        self.print_tunable_params()

if __name__ == "__main__":
    r80_motor = R80MotorControl(axis_num = 0)
    #r80_motor.configure()
    
    print("CONDUCTING MOTOR TESTS")
    print("Placing motor in close loop control. If you move motor, motor will "
          "resist you.")
    
    r80_motor.set_tunable_params(45, 0.004, 0.01, 500, 1500)
    
    r80_motor.mode_close_loop_control()
    
    start_liveplotter(lambda: [r80_motor.odrive_axis.encoder.pos_estimate,
                               r80_motor.odrive_axis.controller.input_pos])
    
    test_selection = [0, 1, 0]

    if test_selection[0] == 1:
        print("Position control")
        r80_motor.odrive_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        
        for angle in [0, 45, 90, 180, 90, 45, 0]:
            print("    Setting motor to {} degrees.".format(angle))
            r80_motor.move_input_pos(angle)
            time.sleep(1)
        
        for i in range(0, 361*2, 10):
            angle = 360*math.sin(math.radians(i))
            print("    Setting motor to {} degrees.".format(angle))
            r80_motor.move_input_pos(angle)
            time.sleep(0.1)
        """
        # Faceoff mode
        tp = 60/83.0
        time.sleep(1)
        r80_motor.move_input_pos(1500)
        print("IT'S ABOUT DRIVE")
        time.sleep(tp)
        r80_motor.move_input_pos(0)
        print("IT's ABOUT POWER")
        time.sleep(tp)
        r80_motor.move_input_pos(1500)
        print("WE STAY HUNGRY")
        time.sleep(tp)
        r80_motor.move_input_pos(0)
        print("WE DEVOUR")
        time.sleep(tp)
        r80_motor.move_input_pos(1500)
        print("PUT IN THE WORK")
        time.sleep(tp)
        r80_motor.move_input_pos(0)
        print("PUT IN THE HOURS")
        time.sleep(tp)
        r80_motor.move_input_pos(1500)
        print("AND TAKE WHATS OURS")
        time.sleep(tp)"""
    
    if test_selection[1] == 1:      
        print("Ramped position control")
        cmd_freq = 10 # Hz
        r80_motor.odrive_axis.controller.config.input_filter_bandwidth = cmd_freq * 3 
        r80_motor.odrive_axis.controller.config.input_mode = INPUT_MODE_POS_FILTER

        for i in range(0, 361*2, 10):
            angle = 360*math.sin(math.radians(i))
            print("    Setting motor to {} degrees.".format(angle))
            r80_motor.move_input_pos(angle)
            time.sleep(1 / cmd_freq)
        r80_motor.odrive_axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        time.sleep(10)
        
    if test_selection[2] == 1:      
        print("Velocity control..")
        r80_motor.odrive_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        for velocity in [1*360, 3*360, 5*360, 7*360, 10*360]:
            print("    Setting motor to {} degrees/s.".format(velocity))
            r80_motor.move_input_vel(velocity)
            time.sleep(1)
        
    print("Idle")
    r80_motor.mode_idle()
    
    print("Done.")
    
    print("Placing motor in idle. If you move motor, motor will "
          "move freely")
    r80_motor.mode_idle()
    
    