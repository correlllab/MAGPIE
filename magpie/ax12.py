#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table ADDRess for AX-12
# EEPROM REGISTER ADDRESSES - Permanently stored in memory once changed
ADDR_AX_MODEL_NUMBER_L = 0
ADDR_AX_MODEL_NUMBER_H = 1
ADDR_AX_VERSION = 2
ADDR_AX_ID = 3
ADDR_AX_BAUD_RATE = 4
ADDR_AX_RETURN_DELAY_TIME = 5
ADDR_AX_CW_ANGLE_LIMIT_L = 6
ADDR_AX_CW_ANGLE_LIMIT_H = 7
ADDR_AX_CCW_ANGLE_LIMIT_L = 8
ADDR_AX_CCW_ANGLE_LIMIT_H = 9
ADDR_AX_SYSTEM_DATA2 = 10
ADDR_AX_LIMIT_TEMPERATURE = 11
ADDR_AX_MIN_LIMIT_VOLTAGE = 12
ADDR_AX_MAX_LIMIT_VOLTAGE = 13
ADDR_AX_MAX_TORQUE_L = 14
ADDR_AX_MAX_TORQUE_H = 15
ADDR_AX_RETURN_LEVEL = 16
ADDR_AX_ALARM_LED = 17
ADDR_AX_ALARM_SHUTDOWN = 18
ADDR_AX_OPERATING_MODE = 19
ADDR_AX_DOWN_CALIBRATION_L = 20
ADDR_AX_DOWN_CALIBRATION_H = 21
ADDR_AX_UP_CALIBRATION_L = 22
ADDR_AX_UP_CALIBRATION_H = 23

# RAM REGISTER ADDRESSES - resets after shut down
ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_LED = 25
ADDR_AX_CW_COMPLIANCE_MARGIN = 26
ADDR_AX_CCW_COMPLIANCE_MARGIN = 27
ADDR_AX_CW_COMPLIANCE_SLOPE = 28
ADDR_AX_CCW_COMPLIANCE_SLOPE = 29
ADDR_AX_GOAL_POSITION_L = 30
ADDR_AX_GOAL_POSITION_H = 31
ADDR_AX_GOAL_SPEED_L = 32
ADDR_AX_GOAL_SPEED_H = 33
ADDR_AX_TORQUE_LIMIT_L = 34
ADDR_AX_TORQUE_LIMIT_H = 35
ADDR_AX_PRESENT_POSITION_L = 36
ADDR_AX_PRESENT_POSITION_H = 37
ADDR_AX_PRESENT_SPEED_L = 38
ADDR_AX_PRESENT_SPEED_H = 39
ADDR_AX_PRESENT_LOAD_L = 40
ADDR_AX_PRESENT_LOAD_H = 41
ADDR_AX_PRESENT_VOLTAGE = 42
ADDR_AX_PRESENT_TEMPERATURE = 43
ADDR_AX_REGISTERED_INSTRUCTION = 44
ADDR_AX_PAUSE_TIME = 45
ADDR_AX_MOVING = 46
ADDR_AX_LOCK = 47
ADDR_AX_PUNCH_L = 48
ADDR_AX_PUNCH_H = 49


class Ax12:
    """ Class for Dynamixel AX12A motors.

    """

    PROTOCOL_VERSION = 1.0
    BAUDRATE = 1_000_000  # Dynamixel default baudrate
    DEVICENAME = '/dev/ttyUSB0'  # e.g 'COM3' windows or '/dev/ttyUSB0' for linux
    DEBUG = True

    def __init__(self, motor_id):
        """Initialize motor with id"""
        self.id = motor_id

    def __repr__(self):
        return "Ax12('{}')".format(self.id)

    # functions to read/write to registers
    def set_register1(self, reg_num, reg_value):
        dxl_comm_result, dxl_error = Ax12.packetHandler.write1ByteTxRx(
            Ax12.portHandler, self.id, reg_num, reg_value)
        Ax12.check_error(dxl_comm_result, dxl_error)

    def get_register1(self, reg_num):
        reg_data, dxl_comm_result, dxl_error = Ax12.packetHandler.read1ByteTxRx(
            Ax12.portHandler, self.id, reg_num)
        Ax12.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def set_register2(self, reg_num, reg_value):
        dxl_comm_result, dxl_error = Ax12.packetHandler.write2ByteTxRx(
            Ax12.portHandler, self.id, reg_num, reg_value)
        Ax12.check_error(dxl_comm_result, dxl_error)

    def get_register2(self, reg_num_low):
        reg_data, dxl_comm_result, dxl_error = Ax12.packetHandler.read2ByteTxRx(
            Ax12.portHandler, self.id, reg_num_low)
        Ax12.check_error(dxl_comm_result, dxl_error)
        return reg_data

    # functions for Read-Only registers
    def get_model_number(self):
        return self.get_register2(ADDR_AX_MODEL_NUMBER_L)

    def get_firmware_version(self):
        return self.get_register1(ADDR_AX_VERSION)

    def get_present_position(self):
        return self.get_register2(ADDR_AX_PRESENT_POSITION_L)

    def get_present_speed(self):
        """Returns 0 if motor is not moving"""
        return self.get_register2(ADDR_AX_PRESENT_SPEED_L)

    def get_load(self):
        return self.get_register2(ADDR_AX_PRESENT_LOAD_L)

    def get_temperature(self):
        """Returns internal temperature in units of Celsius."""
        return self.get_register2(ADDR_AX_PRESENT_TEMPERATURE)

    def get_voltage(self):
        """Returns current voltage supplied to Motor in units of Volts."""
        return self.get_register1(ADDR_AX_PRESENT_VOLTAGE) / 10

    def is_registered(self):
        return self.get_register1(ADDR_AX_REGISTERED_INSTRUCTION)

    def is_moving(self):
        """Returns 1 if motor is moving , 0 if not moving"""
        return self.get_register1(ADDR_AX_MOVING)

    # functions for EEPROM Read/Write registers - stored in memory once changed
    def get_id(self):
        return self.get_register1(ADDR_AX_ID)

    def set_id(self, motor_id):
        self.set_register1(ADDR_AX_ID, motor_id)

    def get_baudrate(self):
        return self.get_register1(ADDR_AX_BAUD_RATE)

    def set_baudrate(self, baudrate):
        
        self.set_register1(ADDR_AX_BAUD_RATE, baudrate)

        if self.DEBUG:
            self.print_status("Baudrate of ", self.id, self.get_baudrate())

    def get_return_delay_time(self):
        return self.get_register1(ADDR_AX_RETURN_DELAY_TIME)

    def set_return_delay_time(self, delay_time):
        self.set_register1(ADDR_AX_RETURN_DELAY_TIME, delay_time)

    def get_cw_angle_limit(self):
        return self.get_register2(ADDR_AX_CW_ANGLE_LIMIT_L)

    def set_cw_angle_limit(self, angle_limit):
        """Sets the lower limit of motor angle [512-0]"""
        self.set_register2(ADDR_AX_CW_ANGLE_LIMIT_L, angle_limit)
        if self.DEBUG:
            self.print_status("cw angle limit of ", self.id, self.get_cw_angle_limit())

    def get_ccw_angle_limit(self):
        return self.get_register2(ADDR_AX_CCW_ANGLE_LIMIT_L)

    def set_ccw_angle_limit(self, angle_limit):
        """Sets the upper limit of motor angle [512-1023]"""
        self.set_register2(ADDR_AX_CCW_ANGLE_LIMIT_L, angle_limit)
        if self.DEBUG:
            self.print_status("ccw angle limit of ", self.id, self.get_ccw_angle_limit())

    def get_min_voltage_limit(self):
        return self.get_register1(ADDR_AX_MIN_LIMIT_VOLTAGE)

    def set_min_voltage_limit(self, min_limit):
        self.set_register1(ADDR_AX_MIN_LIMIT_VOLTAGE, min_limit)

    def get_max_voltage_limit(self):
        return self.get_register1(ADDR_AX_MAX_LIMIT_VOLTAGE)

    def set_max_voltage_limit(self, max_limit):
        self.set_register1(ADDR_AX_MAX_LIMIT_VOLTAGE, max_limit)

    def get_max_torque(self):
        return self.get_register2(ADDR_AX_MAX_TORQUE_L)

    def set_max_torque(self, max_torque):
        self.set_register2(ADDR_AX_MAX_TORQUE_L, max_torque)

    def get_status_return_level(self):
        return self.get_register1(ADDR_AX_RETURN_LEVEL)

    def set_status_return_level(self, return_level):
        self.set_register1(ADDR_AX_RETURN_LEVEL, return_level)

    def get_alarm_led(self):
        return self.get_register1(ADDR_AX_ALARM_LED)

    def set_alarm_led(self, alarm_led):
        self.set_register1(ADDR_AX_ALARM_LED, alarm_led)

    def get_shutdown(self):
        return self.get_register1(ADDR_AX_ALARM_SHUTDOWN)

    def set_shutdown(self, shutdown_value):
        self.get_register1(ADDR_AX_ALARM_SHUTDOWN, shutdown_value)

    # functions for RAM Read/Write registers - resets after shutdown
    def get_torque_enable(self):
        return self.get_register1(ADDR_AX_TORQUE_ENABLE)

    def set_torque_enable(self, torque_bool):
        """ set torque on/off 

        """
        self.set_register1(ADDR_AX_TORQUE_ENABLE, torque_bool)
        
        if self.DEBUG: 
            self.print_status("Torque enable ", self.id, self.get_torque_enable())




    def set_led(self, led_bool):
        """Sets Motor Led; 0 => OFF  1 => ON ."""
        self.set_register1(ADDR_AX_LED, led_bool)

    def get_cw_compliance_margin(self):
        return self.get_register1(ADDR_AX_CW_COMPLIANCE_MARGIN)

    def set_cw_compliance_margin(self, comp_margin):
        self.set_register1(ADDR_AX_CW_COMPLIANCE_MARGIN, comp_margin)

    def get_ccw_compliance_margin(self):
        return self.get_register1(ADDR_AX_CCW_COMPLIANCE_MARGIN)

    def set_ccw_compliance_margin(self, comp_margin):
        self.set_register1(ADDR_AX_CW_COMPLIANCE_MARGIN, comp_margin)

    def get_cw_compliance_slope(self):
        return self.get_register1(ADDR_AX_CW_COMPLIANCE_SLOPE)

    def set_cw_compliance_slope(self, comp_slope):
        self.set_register1(ADDR_AX_CW_COMPLIANCE_SLOPE, comp_slope)

    def get_ccw_compliance_slope(self):
        return self.get_register1(ADDR_AX_CCW_COMPLIANCE_SLOPE)

    def set_ccw_compliance_slope(self, comp_slope):
        self.set_register1(ADDR_AX_CW_COMPLIANCE_SLOPE, comp_slope)

    def get_goal_position(self):
        return self.get_register2(ADDR_AX_GOAL_POSITION_L)

    def set_goal_position(self, goal_pos):
        """Write goal position."""
        self.set_register2(ADDR_AX_GOAL_POSITION_L, goal_pos)

        if self.DEBUG: 
            self.print_status("Position of ", self.id, self.get_goal_position())

    def get_moving_speed(self):
        """Returns moving speed to goal position [0-1023]."""
        return self.get_register2(ADDR_AX_GOAL_SPEED_L)

    def set_moving_speed(self, moving_speed):
        """Set the moving speed to goal position [0-1023]."""
        self.set_register2(ADDR_AX_GOAL_SPEED_L, moving_speed)
        
        if self.DEBUG:
            self.print_status("Moving speed of ", self.id, self.get_moving_speed())


    def get_torque_limit(self):
        return self.get_register2(ADDR_AX_TORQUE_LIMIT_L)

    def set_torque_limit(self, torque_limit):
        self.set_register2(ADDR_AX_TORQUE_LIMIT_L, torque_limit)

    def get_lock(self):
        return self.get_register1(ADDR_AX_LOCK)

    def set_lock(self, lock_val):
        self.set_register1(ADDR_AX_LOCK, lock_val)

    def get_punch(self):
        return self.get_register1(ADDR_AX_PUNCH_L)

    def set_punch(self, punch_val):
        self.set_register1(ADDR_AX_PUNCH_L, punch_val)

    # other functions
    def enable_torque(self):
        """Enable torque for motor."""
        self.set_register1(ADDR_AX_TORQUE_ENABLE, 1)
        if self.DEBUG:
            print("Torque has been successfully enabled for dxl ID: %d" % self.id) 


    def disable_torque(self):
        """Disable torque."""
        self.set_register1(ADDR_AX_TORQUE_ENABLE, 0)
        if self.DEBUG: 
            print("Torque has been successfully disabled for dxl ID: %d" % self.id)


    @classmethod
    def open_port(cls):
        cls.portHandler = PortHandler(cls.DEVICENAME)
        if cls.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

    @classmethod
    def set_baudrate(cls):
        if cls.portHandler.setBaudRate(cls.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

    @classmethod
    def connect(cls):
        cls.open_port()
        cls.set_baudrate()
        cls.packetHandler = PacketHandler(cls.PROTOCOL_VERSION)

    @classmethod
    def disconnect(cls):
        # Close port
        cls.portHandler.closePort()
        print('Successfully closed port')

    @staticmethod
    def check_error(comm_result, dxl_err):
        if comm_result != COMM_SUCCESS:
            print("%s" % Ax12.packetHandler.getTxRxResult(comm_result))
        elif dxl_err != 0:
            print("%s" % Ax12.packetHandler.getRxPacketError(dxl_err))

    @staticmethod
    def raw2deg(delta_raw):
        return round(delta_raw*(300/1023),2)

    @staticmethod
    def deg2raw(delta_deg):
        return int(delta_deg*(1023/300))

    @staticmethod
    def print_status(dxl_property, dxl_id, value):
            print(dxl_property +  "dxl ID: %d set to %d " % (dxl_id, value))
