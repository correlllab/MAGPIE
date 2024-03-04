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
import itertools

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
        self.Crank = 45 # crank length
        self.Finger= 80 # finger length
        self.OffsetCamera2Crank = 38 # difference between camera's x position and servo motor
        self.OffsetCrank2Finger = 24.32 # difference between crank's x position and finger base's x position
        self.servojoint = 84
        #plug in last reference frame to camera position for z axis
        self.Camera2Ref = 63

        # renaming all the above variables to match Stephen Otto's thesis (for my sanity)
        # see p11 of his thesis. all values in mm
        self.crank_length    = 45
        self.finger_length   = 80
        self.offset_servo_x  = 38 # difference between camera x-pos and servo x-pos
        self.offset_servo_y  = -21 # difference between camera y-pos and servo y-pos
        self.offset_finger_x = -24.32 # difference between crank x-pos and finger base x-pos
        self.offset_finger_y = 1.32 # difference between crank y-pos and finger base y-pos

    #this is before you attach your motors to the gripper
    def setup(self):
        self.Finger1.set_goal_position(0)
        self.Finger2.set_goal_position(1023)

    def reset_parameters(self):
        self.apply_to_fingers('set_torque_limit', self.default_parameters['torque'], finger='both', noarg=False)
        self.apply_to_fingers('set_moving_speed', self.default_parameters['speed'], finger='both', noarg=False)
        self.apply_to_fingers('set_cw_compliance_margin', self.default_parameters['compliance_margin'], finger='both', noarg=False)
        self.apply_to_fingers('set_ccw_compliance_margin', self.default_parameters['compliance_margin'], finger='both', noarg=False)
        self.apply_to_fingers('set_cw_compliance_slope', self.default_parameters['compliance_slope'], finger='both', noarg=False)
        self.apply_to_fingers('set_ccw_compliance_slope', self.default_parameters['compliance_slope'], finger='both', noarg=False)
        self.open_gripper()
        time.sleep(0.0025)

    def open_gripper(self):
        open1 = int((self.Finger1theta_min+4)*1023/300)
        open2 = int((self.Finger2theta_max-4)*1023/300)
        self.Finger1.set_goal_position(open1)
        self.Finger2.set_goal_position(open2)
        time.sleep(0.1) # 100ms

    def close_gripper(self):
        close1 = int((self.Finger1theta_max-4)*1023/300)
        close2 = int((self.Finger2theta_min+4)*1023/300)
        self.Finger1.set_goal_position(close1)
        self.Finger2.set_goal_position(close2)
        time.sleep(0.1) # 100ms

    def reset_packet_overload(self, finger='both'):
        self.Finger1.set_torque_enable(True)
        self.Finger2.set_torque_enable(True)
        self.Finger1.set_torque_limit(self.default_parameters['torque'])
        self.Finger2.set_torque_limit(self.default_parameters['torque'])

    def theta_limit(self, delta_theta):
        Motor1_theta = -delta_theta + self.Motor1theta_90
        Motor2_theta = delta_theta + self.Motor2theta_90
        if (Motor1_theta > self.Motor1theta_max or  Motor1_theta < self.Motor1theta_min):
            return 1
        if (Motor2_theta > self.Motor2theta_max or  Motor2_theta < self.Motor2theta_min):
            return 1
        return 0

    # general flow: desired mm distance
    # --> call distance_to_theta(mm)
    # --> call position(theta) to move gripper

    # only for left or right finger, not both
    def theta_to_position(self, delta_theta, finger='left', debug=False):
        sign = (-1.0 if finger=='left' else 1.0)
        parallel_constant = (self.Finger1theta_90 if finger=='left' else self.Finger2theta_90)
        theta = (delta_theta * sign) + parallel_constant
        pos = int(theta * 1023 / 300)
        if debug:
            print(f'position: {pos}')
        return pos

    # inverse methods

    # only for left or right finger, not both
    def position_to_theta(self, position, finger='left'):
        sign = (-1.0 if finger=='left' else 1.0)
        parallel_constant = (self.Finger1theta_90 if finger=='left' else self.Finger2theta_90)
        theta = sign * ((position * 300 / 1023.0) - parallel_constant)
        return theta

    '''
    re-implementation of the above methods, but following Stephen Otto's thesis strictly
    general problem: grasping an object of known width
                     width = aperture * 2 (if closing fingers the same)
                     more generally: width = aperture_f1 + aperture_f2
    goal: want to find the z-offset from the camera center to the finger tip
          when the fingers are grasping the object, ie at the specified aperture
    flow: either z_offset = aperture_to_z(width / 2.0)
          or     z_offset = aperture_to_z(aperture_fx)
    '''

    # aperture to z-offset
    # aperture: (x-axis distance from finger to camera center)
    # z_offset: (z-axis distance from camera center to finger tip, or y_fingertip)
    def aperture_to_z(self, aperture, finger='both', debug=False):
        aperture = (aperture / 2.0) if finger=='both' else aperture
        return self.theta_to_z(self.aperture_to_theta(aperture), debug=debug)

    def set_goal_aperture(self, aperture, finger='both', debug=False, record_load=True):
        aperture = (aperture / 2.0) if finger=='both' else aperture
        # delta_ticks = self.theta_to_position(
        #               self.aperture_to_theta(
        #               np.abs(aperture - self.get_aperture(finger=finger))))
        # each tick (0.29 deg) at speed 100 (~11 rpm) takes 4.4ms to actuate + 1ms buffer
        # wait_time = delta_ticks * self.delay * 2.0 # fat buffer for now
        if record_load:
            return self.set_goal_aperture_record_load(aperture, finger=finger, debug=debug)
        theta = self.aperture_to_theta(aperture)
        if finger=='both':
            self.Finger1.set_goal_position(self.theta_to_position(theta, finger='left', debug=debug))
            self.Finger2.set_goal_position(self.theta_to_position(theta, finger='right', debug=debug))
        elif finger=='left':
            self.Finger1.set_goal_position(self.theta_to_position(theta, finger='left', debug=debug))
        elif finger=='right':
            self.Finger2.set_goal_position(self.theta_to_position(theta, finger='right', debug=debug))
        # time.sleep(wait_time)

    # return aperture, but with math.cos
    def theta_to_aperture(self, theta):
        # movement = np.cos(np.radians(theta)) * self.crank_length
        movement = np.sin(np.radians(theta)) * self.crank_length
        aperture = movement + self.offset_finger_x + self.offset_servo_x
        return aperture

    # return theta, but with math.cos
    def aperture_to_theta(self, aperture):
        # not gonna do the aperture / 2 that distance_to_theta does, but making a note of it
        # note that then aperture means the x-distance from the finger to the x-center of the camera
        # TODO: make consistent across functions, after testing
        movement = aperture - self.offset_finger_x - self.offset_servo_x
        # theta = np.degrees(np.arccos(movement/self.crank_length))
        theta = np.degrees(np.arcsin(movement/self.crank_length))
        return theta

    # return parallel z-distance from camera center to finger tip
    # alternatively, y_fingertip
    def theta_to_z(self, theta, debug=False):
        # l2 = np.sin(np.radians(theta)) * self.crank_length
        l2 = np.cos(np.radians(theta)) * self.crank_length
        l1 = self.finger_length + self.offset_finger_y
        y_fingertip = l2 + l1 + self.offset_servo_y
        if debug:
            print(f'theta: {theta}, l2: {l2}, l1: {l1}, y_fingertip: {y_fingertip}')
        return y_fingertip

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

    # setters
    def set_force(self, force, finger='both', debug=False):
        # convert N to unitless load value
        load = int(self.N_to_load(force))
        if debug:
            print(f'converted load: {load}')

        self.set_torque(load, finger=finger)

    def set_torque(self, torqueLimit, finger='both'):
        self.apply_to_fingers('set_torque_limit', torqueLimit, finger=finger, noarg=False)

    def set_speed(self, speedLimit, finger='both'):
        self.apply_to_fingers('set_moving_speed', speedLimit, finger=finger, noarg=False)

    def set_compliance(self, margin, flexibility, finger='both', debug=False):
        # margin_ax12 = self.theta_to_position(self.distance_to_theta(margin), finger=finger)
        # 1mm of margin ~ 4.5 position units (empirically determined)
        # 0 - 10mm aperture: 44 units, 10 - 20mm aperture: 44 units
        # 20 - 30mm aperture: 45 units, 30 - 40mm aperture: 49 units
        margin_ax12 = int(margin * 4.5)
        flexibility_ax12 = np.power(2, flexibility)
        if debug:
            print(f'margin_ax12: {margin_ax12}')
            print(f'flexibility_ax12: {flexibility_ax12}')
        self.apply_to_fingers('set_cw_compliance_margin', margin_ax12, finger=finger, noarg=False)
        self.apply_to_fingers('set_ccw_compliance_margin', margin_ax12, finger=finger, noarg=False)
        self.apply_to_fingers('set_cw_compliance_slope', flexibility_ax12, finger=finger, noarg=False)
        self.apply_to_fingers('set_ccw_compliance_slope', flexibility_ax12, finger=finger, noarg=False)

    # getters
    def get_position(self, finger='both'):
        return(self.apply_to_fingers('get_present_position', None, finger=finger, noarg=True))

    def get_goal_distance(self, finger='both'):
        if finger=='both':
            return self.goal_distance_both
        else:
            return self.goal_distance_f1 if finger=='left' else self.goal_distance_f2

    def get_aperture(self, finger='both'):
        '''
        @return aperture in mm, either between both fingers, or from finger to x-center
        '''
        # perform inverse calculations of theta_to_position, distance_to_theta
        f1_theta = self.position_to_theta(self.get_position(finger='left'), finger='left')
        f2_theta = self.position_to_theta(self.get_position(finger='right'), finger='right')
        f1_aperture  = self.theta_to_aperture(f1_theta)
        f2_aperture  = self.theta_to_aperture(f2_theta)
        if finger == 'both':
            return f1_aperture + f2_aperture
        else:
            return f1_aperture if finger=='left' else f2_aperture

    def get_temp(self, finger='both'):
        return self.apply_to_fingers('get_temperature', None, finger=finger, noarg=True)

    def get_load(self, finger='both'):
        return self.apply_to_fingers('get_load', None, finger=finger, noarg=True)

    def get_torque(self, finger='both'):
        return self.apply_to_fingers('get_torque_limit', None, finger=finger, noarg=True)

    def get_compliance(self, finger='both'):
        return [self.apply_to_fingers('get_cw_compliance_margin', None, finger=finger, noarg=True),
                self.apply_to_fingers('get_ccw_compliance_margin', None, finger=finger, noarg=True),
                self.apply_to_fingers('get_cw_compliance_slope', None, finger=finger, noarg=True),
                self.apply_to_fingers('get_ccw_compliance_slope', None, finger=finger, noarg=True)
        ]

    # gripper motion
    def close_until_contact_force(self, stop_aperture, stop_force, finger='both', debug='False'):
        stop_load = self.N_to_load(stop_force)
        stop_ax12 = None
        delta = None
        sign = None
        curr_pos = self.get_position(finger=finger)
        if finger=='both':
            stop_ax12 = [
                self.theta_to_position(self.aperture_to_theta(stop_aperture), finger='left', debug=debug ),
                self.theta_to_position(self.aperture_to_theta(stop_aperture), finger='right', debug=debug )
            ]
            delta = np.array(stop_ax12) - np.array(curr_pos)
        else:
            stop_ax12 = self.theta_to_position(self.aperture_to_theta(stop_aperture), finger=finger, debug=debug)
            delta = stop_ax12 - curr_pos
        sign = np.sign(delta)

        if finger=='both':
            p1 = threading.Thread(target=self.close_until_contact_force_helper, args=(delta[0], stop_load, sign[0], 'left', debug))
            p2 = threading.Thread(target=self.close_until_contact_force_helper, args=(delta[1], stop_load, sign[1], 'right', debug))
            p1.start()
            p2.start()
            self.goal_distance_both = self.get_distance(finger='both')
        else:
            self.close_until_contact_force_helper(delta, stop_ax12, sign, finger=finger, debug=debug)

    # only for left or right finger, not both
    def close_until_contact_force_helper(self, stop_pos, stop_load, sign, finger='left', debug=False):
        finger_ax12 = self.Finger1 if finger=='left' else self.Finger2
        curr_pos = finger_ax12.get_present_position()
        time.sleep(self.latency)
        for next_pos in range(curr_pos, stop_pos, sign * 1):
            finger_ax12.set_goal_position(next_pos)
            time.sleep(self.delay * 2)
            curr_pos = finger_ax12.get_present_position()
            curr_load = finger_ax12.get_load()
            time.sleep(self.latency)
            if curr_load > stop_load:
                force = self.load_to_N(curr_load)
                distance = self.get_aperture(finger=finger)
                print(f'{finger} finger reached stop force: {force} at distance: {distance}')
                print(f'{finger} finger reached stop load: {curr_load} at position: {curr_pos}')
                # todo: set goal_distance_fx class variable, but not really necessary
                finger_ax12.set_goal_position(curr_pos)
                break

    def set_goal_aperture_record_load(self, stop_aperture, finger='both', debug='False'):
        stop_ax12 = None
        delta = None
        sign = None
        curr_pos = self.get_position(finger=finger)
        if finger=='both':
            stop_ax12 = [
                self.theta_to_position(self.aperture_to_theta(stop_aperture), finger='left', debug=debug ),
                self.theta_to_position(self.aperture_to_theta(stop_aperture), finger='right', debug=debug )
            ]
            delta = np.array(stop_ax12) - np.array(curr_pos)
        else:
            stop_ax12 = self.theta_to_position(self.aperture_to_theta(stop_aperture), finger=finger, debug=debug)
            delta = stop_ax12 - curr_pos
        sign = np.sign(delta)

        if finger=='both':
            pos_load_l = [[], []]
            pos_load_r = [[], []]
            self.record_load_both_helper(stop_ax12, sign, [pos_load_l, pos_load_r], debug)
            return pos_load_l, pos_load_r
        else:
            pos_load = [[], []]
            self.record_load_helper(stop_ax12, sign, pos_load, finger=finger, debug=debug)
            return pos_load

    # only for left or right finger, not both
    def record_load_helper(self, stop_pos, sign, pld, finger='left', debug=False):
        '''
        @param stop_pos: position to stop at
        @param sign: direction to move in
        @param pld: position-load data array
        '''
        finger_ax12 = self.Finger1 if finger=='left' else self.Finger2
        curr_pos = finger_ax12.get_present_position()
        time.sleep(self.latency)
        for next_pos in range(curr_pos, stop_pos, sign * 1):
            finger_ax12.set_goal_position(next_pos)
            time.sleep(self.delay * 2)
            curr_pos = finger_ax12.get_present_position()
            curr_load = finger_ax12.get_load()
            time.sleep(self.latency)
            if debug:
                print(f'position: {curr_pos}, load: {curr_load}')
            pld[0].append(curr_pos)
            pld[1].append(curr_load)

    # only for left or right finger, not both
    def record_load_both_helper(self, stop_pos, sign, pld, debug=False):
        '''
        @param stop_pos: position to stop at
        @param sign: direction to move in
        @param pld: position-load data array
        '''
        curr_pos = self.get_position(finger='both')
        time.sleep(self.latency)
        lrange = range(curr_pos[0], stop_pos[0], sign[0] * 1)
        rrange = range(curr_pos[1], stop_pos[1], sign[1] * 1)
        # get stop position of shorter range
        stub = stop_pos[0] if len(lrange) < len(rrange) else stop_pos[1]
        for next_pos_l, next_pos_r in itertools.zip_longest(lrange, rrange, fillvalue=stub):
            self.Finger1.set_goal_position(next_pos_l)
            self.Finger2.set_goal_position(next_pos_r)
            time.sleep(self.delay * 2)
            curr_pos = self.get_position(finger='both')
            curr_load = self.get_load(finger='both')
            time.sleep(self.delay)
            if debug:
                print(f'left position: {curr_pos[0]}, load: {curr_load[1]}')
                print(f'right position: {curr_pos[1]}, load: {curr_load[1]}')
            pld[0][0].append(curr_pos[0])
            pld[1][0].append(curr_pos[1])
            pld[0][1].append(curr_load[0])
            pld[1][1].append(curr_load[1])

    def contact_force_met(self, pos_load, stop_force, finger='both'):
        '''
        @param pos_load: position-load data array of shape (2, n) --> [[positions], [loads]]
        @param stop_load: force to stop at in N
        @param finger: left, right, or both fingers
        @return: True if stop_load is met at any point in pos_load, False otherwise
        '''
        stop_load = self.N_to_load(stop_force)
        return stop_load in pos_load[1]

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
            N = 0.0025 * load - 0.0000007 * load**2
        return N

    def N_to_load(self, N):
        '''
        @param N: force normal load at finger contact in N
        '''
        # derived by Stephen Otto empirically
        # see eqn on p17, figure 14 on p18 of: https://www.proquest.com/docview/2868478510?%20
        # invert load_to_N
        low_load = N < 0.25 
        a = -0.00001889 if not low_load else -0.0000007
        b = 0.038399 if not low_load else 0.0025
        c = (-3.4073 - N) if not low_load else (-N)
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return None  # No real solution
        else:
            load = (-b + math.sqrt(discriminant)) / (2*a)
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