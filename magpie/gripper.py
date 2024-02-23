import sys
sys.path.append("../../")
from magpie.ax12 import Ax12
import math
import spatialmath as sm
import copy
import time
import numpy as np
from multiprocessing import Process
import threading

class Gripper:
    def __init__(self, servoport = '/dev/ttyACM0'):
        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux, '/dev/ttyACM0'
        # sets baudrate and opens com port
        # Ax12.DEVICENAME = '/dev/cu.usbmodem141401'
        Ax12.DEVICENAME = servoport
        Ax12.BAUDRATE = 1_000_000
        # sets baudrate and opens com port
        Ax12.connect()
        # create AX12 instance with ID 1 and 2
        #Motor ID1 should be on the right with the camera facing you
        finger_id1 = 1 # left gripper
        finger_id2 = 2
        self.Finger1 = Ax12(finger_id1)
        self.Finger2 = Ax12(finger_id2)
        #speed is in bits from 0-1023 for CCW; 1024 -2047 CW
        self.speed = 100 # about 10% speed, or 11rpm
        self.Finger1.set_moving_speed(self.speed)
        self.Finger2.set_moving_speed(self.speed)
        # torque in bits from 0-1023
        self.torque = 200
        self.goal_distance_both = 0 # in mm
        self.goal_distance_f1 = 0
        self.goal_distance_f2 = 0

        '''
        default latency (there and back) is 500 us, or 0.5 ms
        actuating one tick at a time (0.29 deg)
        self.speed = 100, which is in wheel mode
        equal to ~11 rpm = 3960 deg/m
        so one tick takes 4.4ms to actuate
        actuation time + latency = 4.9ms, give some buffer
        '''
        self.delay = 0.0055
        self.latency = 0.0006

        self.Finger1.set_torque_limit(self.torque)
        self.Finger2.set_torque_limit(self.torque)
        #input motor theta max and measurements by using dynamixel
        self.Finger1theta_max = 176
        self.Finger1theta_min = 85
        self.Finger2theta_max = 218
        self.Finger2theta_min = 128
        # set bar parallel to camera(Input)
        self.Finger1theta_90 = 150
        # set bar parallel to camera(Input)
        self.Finger2theta_90 = 155

        self.default_parameters = {
                'torque': 200,
                'speed': 100,
                'compliance_margin': 1,
                'compliance_slope': 32,
            }

        #dont touch
        self.Crank = 45
        self.Finger= 80 # don't know what this measurement is. finger length probably?
        self.OffsetCamera2Crank = 38
        self.OffsetCrank2Finger = 24.32
        self.servojoint = 84
        #plug in last reference frame to camera position for z axis
        self.Camera2Ref = 63

    #this is before you attach your motors to the gripper
    def setup(self):
        self.Finger1.set_goal_position(0)
        self.Finger2.set_goal_position(1023)

    def reset_parameters(self):
        self.apply_to_fingers('set_torque_limit', self.default_parameters['torque'], finger='both', noarg=False)
        self.apply_to_fingers('set_moving_speed', self.default_parameters['speed'], finger='both', noarg=False)
        self.apply_to_fingers('set_compliance_margin', self.default_parameters['compliance_margin'], finger='both', noarg=False)
        self.apply_to_fingers('set_compliance_slope', self.default_parameters['compliance_slope'], finger='both', noarg=False)
        time.sleep(0.0025)

    def open_gripper(self):
        open1 = int((self.Motor1theta_min+4)*1023/300)
        open2 = int((self.Motor2theta_max-4)*1023/300)
        self.Finger1.set_goal_position(open1)
        self.Finger2.set_goal_position(open2)
        time.sleep(0.1) # 100ms

    def close_gripper(self):
        close1 = int((self.Motor1theta_max-4)*1023/300)
        close2 = int((self.Motor2theta_min+4)*1023/300)
        self.Finger1.set_goal_position(close1)
        self.Finger2.set_goal_position(close2)
        time.sleep(0.1) # 100ms

    def theta_limit(self, delta_theta):
        Motor1_theta = -delta_theta + self.Motor1theta_90
        Motor2_theta = delta_theta + self.Motor2theta_90
        if (Motor1_theta > self.Motor1theta_max or  Motor1_theta < self.Motor1theta_min):
            return 1
        if (Motor2_theta > self.Motor2theta_max or  Motor2_theta < self.Motor2theta_min):
            return 1
        return 0

    def distance_from_ref(self, delta_theta):
        delta_Z = self.Crank*math.cos(math.radians(delta_theta)) - 14 + 81.32 + self.Camera2Ref
        return delta_Z

    def apply_to_fingers(self, action_func_name, arg, finger='both', noarg=False):
        arg = () if noarg else (arg)
        if finger == 'both':
            if noarg:
                return [getattr(self.Finger1, action_func_name)(),
                        getattr(self.Finger2, action_func_name)()]
            else:
                return [getattr(self.Finger1, action_func_name)(arg),
                        getattr(self.Finger2, action_func_name)(arg)]
        elif finger == 'left':
            if noarg:
                return getattr(self.Finger1, action_func_name)()
            else:
                return getattr(self.Finger1, action_func_name)(arg)
        elif finger == 'right':
            if noarg:
                return getattr(self.Finger2, action_func_name)()
            else:
                return getattr(self.Finger2, action_func_name)(arg)

    # general flow: desired mm distance
    # --> call distance_to_theta(mm)
    # --> call position(theta) to move gripper

    def distance_to_theta(self,width):
        xPositionFromCenter = width/2
        #positive movement means it goes away from center(open gripper)
        #negative means it closes the gripper
        Movement = xPositionFromCenter - self.OffsetCamera2Crank + self.OffsetCrank2Finger
        delta_theta = math.degrees(math.asin(Movement/self.Crank))
        #theta comes in degrees
        return delta_theta

    # only for left or right finger, not both
    def theta_to_position(self, delta_theta, finger='left'):
        sign = (-1.0 if finger=='left' else 1.0)
        parallel_constant = (self.Finger1theta_90 if finger=='left' else self.Finger2theta_90)
        theta = (delta_theta * sign) + parallel_constant
        pos = int(theta * 1023 / 300)
        return pos

    # inverse methods

    # only for left or right finger, not both
    def position_to_theta(self, position, finger='left'):
        sign = (-1.0 if finger=='left' else 1.0)
        parallel_constant = (self.Finger1theta_90 if finger=='left' else self.Finger2theta_90)
        theta = (sign * (position * 300 / 1023.0)) - parallel_constant
        return theta

    def theta_to_distance(self, theta):
        movement = self.Crank * math.sin(math.radians(theta))
        distance = movement + self.OffsetCamera2Crank - self.OffsetCrank2Finger
        return distance

    # setters
    def set_goal_distance(self, distance, finger='both'):
        distance = distance if finger=='both' else (distance * 2.0)
        theta = self.theta_to_position(self.distance_to_theta(distance))
        if finger=='both':
            theta = self.distance_to_theta(distance)
            # self.theta_to_position(theta, finger=finger)
            self.Finger1.set_goal_position(self.theta_to_position(theta, finger='left'))
            self.Finger2.set_goal_position(self.theta_to_position(theta, finger='right'))
        elif finger=='left':
            theta = self.distance_to_theta(distance)
            self.Finger1.set_goal_position(self.theta_to_position(theta, finger='left'))
        elif finger=='right':
            theta = self.distance_to_theta(distance)
            self.Finger2.set_goal_position(self.theta_to_position(theta, finger='right'))

    def set_force(self, force, finger='both'):
        # convert N to unitless load value
        load = self.N_to_load(force)
        self.set_torque(load, finger=finger)

    def set_torque(self, torqueLimit, finger='both'):
        self.apply_to_fingers('set_torque_limit', torqueLimit, finger=finger, noarg=False)
        # if finger=='both':
        #     self.Finger1.set_torque_limit(torqueLimit)
        #     self.Finger2.set_torque_limit(torqueLimit)
        # elif finger=='left':
        #     self.Finger1.set_torque_limit(torqueLimit)
        # elif finger=='right':
        #     self.Finger2.set_torque_limit(torqueLimit)

    def set_speed(self, speedLimit, finger='both'):
        self.apply_to_fingers('set_moving_speed', speedLimit, finger=finger, noarg=False)
        # self.Finger1.set_moving_speed(speedLimit)
        # self.Finger2.set_moving_speed(speedLimit)

    def set_compliance(self, margin, flexibility, finger='both'):
        margin_ax12 = self.theta_to_position(self.distance_to_theta(margin), finger=finger)
        flexibility_ax12 = np.power(2, flexibility)
        self.apply_to_fingers('set_compliance_margin', margin_ax12, finger=finger, noarg=False)
        self.apply_to_fingers('set_compliance_slope', flexibility_ax12, finger=finger, noarg=False)
        # self.Finger1.set_compliance_margin(margin)
        # self.Finger1.set_compliance_slope(flexibility)
        # self.Finger2.set_compliance_margin(margin)
        # self.Finger2.set_compliance_slope(flexibility)

    # getters
    def get_position(self, finger='both'):
        return(self.apply_to_fingers('get_present_position', None, finger=finger, noarg=True))
        # self.Finger1.get_present_position()
        # self.Finger2.get_present_position()

    def get_goal_distance(self, finger='both'):
        if finger=='both':
            return self.goal_distance_both
        else:
            return self.goal_distance_f1 if finger=='left' else self.goal_distance_f2

    def get_distance(self, finger='both'):
        '''
        @return distance in mm, either between both fingers, or from finger to x-center
        '''
        # perform inverse calculations of theta_to_position, distance_to_theta
        f1_theta = self.position_to_theta(self.get_position(finger='left'))
        f2_theta = self.position_to_theta(self.get_position(finger='right'))
        f1_dist  = self.theta_to_distance(f1_theta)
        f2_dist  = self.theta_to_distance(f2_theta)
        if finger == 'both':
            return f1_dist + f2_dist
        else:
            return f1_dist if finger=='left' else f2_dist

    def get_temp(self, finger='both'):
        self.apply_to_fingers('get_temperature', None, finger=finger, noarg=True)
        # self.Finger1.get_temperature()
        # self.Finger2.get_temperature()

    def get_load(self, finger='both'):
        self.apply_to_fingers('get_load', None, finger=finger, noarg=True)
        # if finger=='both':
        #     return [self.Finger1.get_load(),
        #             self.Finger2.get_load()]
        # elif finger=='left':
        #     return self.Finger1.get_load()
        # elif finger=='right':
        #     return self.Finger2.get_load()

    def close_until_contact_force(self, stop_position, stop_force, finger='both'):
        stop_load = self.N_to_load(stop_force)
        stop_ax12 = None
        delta = None
        sign = None
        curr_pos = self.get_position(finger=finger)
        if finger=='both':
            stop_ax12 = [
                self.theta_to_position(self.distance_to_theta(stop_position), finger='left'),
                self.theta_to_position(self.distance_to_theta(stop_position), finger='right')
            ]
            delta = np.array(curr_pos) - np.array(stop_ax12)
        else:
            stop_ax12 = self.theta_to_position(self.distance_to_theta(stop_position), finger=finger)
            delta = curr_pos - stop_ax12
        sign = np.sign(delta)

        if finger=='both':
            p1 = threading.Thread(target=self.close_until_contact_force_helper, args=(delta[0], stop_load, sign[0], 'left'))
            p2 = threading.Thread(target=self.close_until_contact_force_helper, args=(delta[1], stop_load, sign[1], 'right'))
            p1.start()
            p2.start()
            self.goal_distance_both = self.get_distance(finger='both')
        else:
            self.close_until_contact_force_helper(delta, stop_load, sign, finger=finger)
    
    # only for left or right finger, not both
    def close_until_contact_force_helper(self, delta, stop_load, sign, finger='left'):
        finger_ax12 = self.Finger1 if finger=='left' else self.Finger2
        curr_pos = finger_ax12.get_present_position()
        time.sleep(self.latency)
        for i in range(0, delta, sign * 1):
            finger_ax12.set_goal_position(curr_pos + i)
            time.sleep(self.delay)
            curr_pos = finger_ax12.get_present_position()
            curr_load = finger_ax12.get_load()
            time.sleep(self.latency)
            if curr_load > stop_load:
                force = self.load_to_N(curr_load)
                distance = self.get_distance(finger=finger)
                print(f'{finger} finger reached stop force: {force} at distance: {distance}')
                print(f'{finger} finger reached stop load: {curr_load} at position: {curr_pos}')
                # todo: set goal_distance_fx class variable, but not really necessary
                finger_ax12.set_goal_position(curr_pos)
                break

    # convert unitless load values to force normal load at gripper contact point
    def load_to_N(self, load):
        '''
        @param load: unitless load value in bits, 0-2048. 0-1023 represents CW load, 1024-2047 represents CCW load
        '''
        # derived by Stephen Otto empirically
        # see eqn on p17, figure 14 on p18 of: https://www.proquest.com/docview/2868478510?%20
        if load > 1023:
            load = load - 1023
        N = -0.00001889 * load**2 + 0.038399 * load - 3.4073
        if load < 100:
            # made up polynomial to approximate the low load region
            N = 0.0025 * load - 0.07 * load**2
        return N

    def N_to_load(self, N):
        '''
        @param N: force normal load at finger contact in N
        '''
        # derived by Stephen Otto empirically
        # see eqn on p17, figure 14 on p18 of: https://www.proquest.com/docview/2868478510?%20
        load = 0.00001889 * N**2 - 0.038399 * N + 3.4073
        if load < 100:
            # made up polynomial to approximate the low load region
            load = 0.0025 * N - 0.07 * N**2
        return load

    def disconnect(self):
        Ax12.disconnect()

if __name__ =="__main__":
    Controller = Gripper()
    Controller.openGripper()
    while True:

        width_object = int(input("Input Width"))
        delta_theta = Controller.distance_to_theta(width_object)
        error = Controller.thetaLimit(delta_theta)
        if error :
            print("error")
            continue
        Dist = Gripper.distance_from_ref(delta_theta)
        Controller.position(delta_theta)