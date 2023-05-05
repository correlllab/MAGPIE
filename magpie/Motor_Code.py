from magpie.Ax12 import Ax12
import math
class Motors:
    def __init__(self,portName = "/dev/ttyACM0"):
        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
        # sets baudrate and opens com port
        Ax12.DEVICENAME = portName
        Ax12.BAUDRATE = 1_000_000
        # sets baudrate and opens com port
        Ax12.connect()
        # create AX12 instance with ID 1 and 2
        #Motor ID1 should be on the right with the camera facing you
        motor_id1 = 1
        motor_id2 = 2
        self.Motor1 = Ax12(motor_id1)
        self.Motor2 = Ax12(motor_id2)
        #speed is in bits from 0-1023 for CCW; 1024 -2047 CW
        self.speed = 100
        self.Motor1.set_moving_speed(self.speed)
        self.Motor2.set_moving_speed(self.speed)
        # torque in bits from 0-1023
        self.torque = 200
        self.Motor1.set_torque_limit(self.torque)
        self.Motor2.set_torque_limit(self.torque)
        #input motor theta max and measurements by using dynamixel
        self.Motor1theta_max = 176
        self.Motor1theta_min = 85
        self.Motor2theta_max = 218
        self.Motor2theta_min = 128
        # set bar parallel to camera(Input)
        self.Motor1theta_90 = 150
        # set bar parallel to camera(Input)
        self.Motor2theta_90 = 155
        #dont touch
        self.Crank = 45
        self.Finger= 80
        self.OffsetCamera2Crank = 38
        self.OffsetCrank2Finger = 24.32
        self.servojoint = 84
        #plug in last reference frame to camera position for z axis
        self.Camera2Ref = 63
        
    #this is before you attach your motors to the gripper       
    def setup(self):
        self.Motor1.set_goal_position(0)
        self.Motor2.set_goal_position(1023)
        
    def position(self, delta_theta):
        Motor1_theta = -delta_theta + self.Motor1theta_90
        Motor2_theta = delta_theta + self.Motor2theta_90
        desiredPosition1 = int(Motor1_theta*1023/(300))
        desiredPosition2 = int(Motor2_theta*1023/(300))
        self.Motor1.set_goal_position(desiredPosition1)
        self.Motor2.set_goal_position(desiredPosition2)
            
    def torquelimit(self, torqueLimit):
        self.Motor1.set_torque_limit(torqueLimit)
        self.Motor2.set_torque_limit(torqueLimit)

    def speedlimit(self, speedLimit):
        self.Motor1.set_moving_speed(speedLimit)
        self.Motor2.set_moving_speed(speedLimit)
    
    def distance2theta(self,width):
        xPositionFromCenter = width/2
        #positive movement means it goes away from center(open gripper)
        #negative means it closes the gripper
        Movement = xPositionFromCenter - self.OffsetCamera2Crank + self.OffsetCrank2Finger
        delta_theta = math.degrees(math.asin(Movement/self.Crank))
        #theta comes in degrees
        return delta_theta
    
    def thetaLimit(self,delta_theta):
        Motor1_theta = -delta_theta + self.Motor1theta_90
        Motor2_theta = delta_theta + self.Motor2theta_90
        if (Motor1_theta > self.Motor1theta_max or  Motor1_theta < self.Motor1theta_min):
            return 1
        if (Motor2_theta > self.Motor2theta_max or  Motor2_theta < self.Motor2theta_min):
            return 1
        return 0
    
    def distanceFromRef(self, delta_theta):
        delta_Z = self.Crank*math.cos(math.radians(delta_theta)) - 14 + 81.32 + self.Camera2Ref
        return deltaZ

    def getPosition(self):
        self.Motor1.get_present_position()
        self.Motor2.get_present_position()
        
    def openGripper(self):
        open1 = int((self.Motor1theta_min+4)*1023/300)
        open2 = int((self.Motor2theta_max-4)*1023/300)
        self.Motor1.set_goal_position(open1)
        self.Motor2.set_goal_position(open2)

    def Temp(self):
        self.Motor1.get_temperature()
        self.Motor2.get_temperature()
        
    def Load(self):
        self.Motor1.get_load()
        self.Motor2.get_load()

if __name__ == "__main__":
    #Initializes everything
    Controller = Motors()
    Controller.openGripper()
    while True:
        width_object = int(input("Input Width"))
        delta_theta = Controller.distance2theta(width_object)
        error = Controller.thetaLimit(delta_theta)
        if error :
            print("error")
            continue
        Dist = distanceFromRef(delta_theta)
        Controller.position(delta_theta)











