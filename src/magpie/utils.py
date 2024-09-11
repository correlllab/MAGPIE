# utils.py
# James Watson , 2019 August for Robotic Materials
# General purpose helper functions for robot capabilities

########## INIT ###################################################################################

##### Imports #####
import time, datetime, threading, subprocess
from queue import Queue
from time import sleep
from time import time as now
from math import floor
import numpy as np
from numpy import sin, cos
import platform
if platform.system() == 'Linux':
    from psutil import sensors_temperatures
# from tcp_latency import measure_latency

##### Aliases #####
np_choice = np.random.choice


########## GEOMETRY FUNCTIONS #####################################################################


def vec_mag(v1):
    """ Return the magnitude of the vector """
    return np.linalg.norm(v1)


def vec_diff_mag(v1, v2):
    """ Return the magnitude of the difference between two vectors """
    return np.linalg.norm(np.subtract(v1, v2))


def vec_unit(vec):
    """ Get the unit vector in the direction of 'vec' """
    return np.divide(vec, np.linalg.norm(vec))  # NOTE: This will throw an error for zero-length 'vec'


def vec_angle_between(v1, v2):
    """ Returns the axis-angle in radians between vectors 'v1' and 'v2' """
    # URL, angle between two vectors: http://stackoverflow.com/a/13849249/893511
    v1_u = vec_unit(v1)
    v2_u = vec_unit(v2)
    if (vec_mag(v1_u) == 0.0) or (vec_mag(v2_u) == 0.0):
        return float('nan')
    angle = np.arccos(np.dot(v1_u, v2_u))
    if np.isnan(angle):
        if (v1_u == v2_u).all():
            return 0.0
        else:
            return np.pi
    return np.cross(v1_u, v2_u), angle


# def np_add(*args):
#     """ Perform 'np.add' on more than two args """
#     if len(args) > 2:  # If there are more than 2 args, add the first arg to recur on remainder of args
#         # Note the star operator is needed for recursive call, unpack to positional args
#         return np.add(args[0], np_add(*args[1:]))
#     else:  # base case, there are 2 args*, use vanilla 'np.add'
#         # *NOTE: This function assumes there are at least two args, if only 1 an error will occur
#         return np.add(args[0], args[1])


# def np_dot(*args):
#     """ Perform 'np.dot' on more than two args """
#     if len(args) > 2:  # If there are more than 2 args, add the first arg to recur on remainder of args
#         # Note the star operator is needed for recursive call, unpack to positional args
#         return np.dot(args[0], np_dot(*args[1:]))
#     else:  # base case, there are 2 args*, use vanilla 'np.add'
#         # *NOTE: This function assumes there are at least two args, if only 1 an error will occur
#         return np.dot(args[0], args[1])


# def np_subtract(*args):
#     """ Perform 'np.subtract' on more than two args """
#     if len(args) > 2:  # If there are more than 2 args, subtract the last arg from the preceeding remainder of args
#         # Note the star operator is needed for recursive call, unpack to positional args
#         return np.subtract(np_subtract(*args[:-1]), args[-1])
#     else:  # base case, there are 2 args*, use vanilla 'np.subtract'
#         # *NOTE: This function assumes there are at least two args, if only 1 an error will occur
#         return np.subtract(args[0], args[1])


def ver(theta):
    """ Versine , radians """
    return 1.0 - cos(theta)


def skew_sym(k):
    """ Return the skew-symmetric matrix for R3 vector `k` """
    return np.array([[0.0, -k[2],  k[1]],
                     [k[2],  0.0, -k[0]],
                     [-k[1],  k[0],  0.0]])


def R_from_k_theta(k, theta):
    """ Get the rotation matrix equivalent to the axis-angle """
    k = np.reshape(vec_unit(k), (3, 1))
    A = np.multiply(np.eye(3), cos(theta))
    B = np.multiply(skew_sym(k), sin(theta))
    C = np.multiply(np.dot(k, np.transpose(k)), ver(theta))
    return A + B + C


def vec_linspace(vec1, vec2, numPts):
    """ Return a list of 'numPts' points (vectors) evenly spaced from 'vec1' to 'vec2', inclusive """
    # NOTE: This function assumes that 'bgnPnt' and 'endPnt' have the same dimensionality
    diff = np.subtract(vec2, vec1)  # Vector from point 1 to point 2
    direction = vec_unit(diff)  # Direction of the vector between the two
    span = vec_mag(diff)  # the Euclidian distance between the two points
    ptsList = []
    for value in np.linspace(0, span, num=numPts):  # For each value in the range
        ptsList.append(np.add(vec1,  # append the sum of the first point and
                              np.multiply(direction,  # a vector that is an additional space along the difference
                                          value)))
    return ptsList


def intersect_pnt_2D(seg1, seg2):
    """ if line segments 'seg1' and 'seg2' intersect, then return intersection point , otherwise return false """
    #               { seg1: [ [x1,y1] , [x2,y2] ] , seg2: [ [x3,y3] , [x4,y4] ]  }
    # URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    den = (seg2[1][1] - seg2[0][1]) * (seg1[1][0] - seg1[0][0]) - \
        (seg2[1][0] - seg2[0][0]) * (seg1[1][1] - seg1[0][1])
    uAnum = (seg2[1][0] - seg2[0][0]) * (seg1[0][1] - seg2[0][1]) - \
        (seg2[1][1] - seg2[0][1]) * (seg1[0][0] - seg2[0][0])
    uBnum = (seg1[1][0] - seg1[0][0]) * (seg1[0][1] - seg2[0][1]) - \
        (seg1[1][1] - seg1[0][1]) * (seg1[0][0] - seg2[0][0])
    if den == 0:
        # If Lines are coincident, return the average of segment centers, this is not overlap center
        if eq(uAnum, 0.0) or eq(uBnum, 0.0):
            return vec_avg(seg1[0], seg1[1], seg2[0], seg2[1])
        # Else Lines are parallel
        else:
            return False
    else:
        uA = uAnum * 1.0 / den
        uB = uBnum * 1.0 / den
        if (uA >= 0 and uA <= 1) and (uB >= 0 and uB <= 1):
            # { seg1:[ [x1,y1] , [x2,y2] ] , seg2: [ [x3,y3] , [x4,y4] ]  }
            #   return [ x1 + uA * ( x2 - x1 ) , y1 + uA * ( y2 - y1 ) ]
            return [seg1[0][0] + uA * (seg1[1][0] - seg1[0][0]), seg1[0][1] + uA * (seg1[1][1] - seg1[0][1])]
        else:
            return False  # Lines do not intersect


def closest_pair(ptsList):
    """ Return the indices of the closest pair of points in the list """
    # FIXME: FAILS FOR LIST OF ONE POINT AND OTHER UNKNOWN CASES
    # NOTE: Linear search, O(n^2)
    infty = float('inf')
    N = len(ptsList)
    leastPair = (None, None)
    leastDist = infty
    for i in range(0, N-1):
        for j in range(i+1, N):
            dist = vec_diff_mag(ptsList[i], ptsList[j])
            if dist < leastDist:
                leastDist = dist
                leastPair = (i, j)
    return leastPair, leastDist

def position_from_pose( pose ):
    """ Return the displacement part of a homogeneous pose matrix """
    return [ pose[i][3] for i in range(3) ]

def combine_poses( rot_pose = None , trans_pose = None ):
    """ Return a pose that is the position of `trans_pose` and the orientation of `rot_pose` """    
    pose = np.ones((4,4))
    pose[:4,:3] = rot_pose[:4,:3]
    pose[:4,3] = trans_pose[:4,3] 
    return pose

def transform_vectors( vectors , pose ):
    """ Transform vectors (directions) by a transformation matrix orientation """
    R = pose[0:3, 0:3]
    return vectors.dot( R.T )

def rotate_vectors( vectors ,
                    rx = 0.0 , ry = 0.0 , rz = 0.0 ,
                    k = np.array([1.0, 0.0, 0.0]) , theta = None ,
                    frame = 'base' ):
    pose = rotate_pose( origin_pose() , 
                        rx = rx , ry = ry , rz = rz ,
                        k = k , theta = theta , 
                        frame = frame )
    return transform_vectors( vectors , pose )

# _ End Geo _



########## STATISTICAL FUNCTIONS ##################################################################


def rand_sample_list_wo_replace(origList, N):
    """ Return a list that is composed of `N` random samples of `origList`, drawn without replacement """
    N = min(len(origList), N)  # Clamp `N` to the actual size of the input array
    rtnLst = []
    indices = np_choice(list(range(len(origList))), N, replace=False)
    for i in indices:
        rtnLst.append(origList[i])
    return rtnLst



########## RECORDING FUNCTIONS ####################################################################


def condition_false():
    """ Always return False """
    return False


def dayTimeStamp(): return datetime.datetime.now().strftime(
    '%Y-%m-%d')  # http://stackoverflow.com/a/5215012/893511


""" Return a formatted timestamp string, useful for logging and debugging """


def nowTimeStamp(): return datetime.datetime.now().strftime(
    '%Y-%m-%d_%H-%M-%S')  # http://stackoverflow.com/a/5215012/893511

""" Return a formatted timestamp string, useful for logging and debugging """


def nowTimeStampFine(): return datetime.datetime.now().strftime(
    '%Y-%m-%d_%H-%M-%S-%f')  # http://stackoverflow.com/a/5215012/893511

""" Return a formatted timestamp string, useful for logging and debugging """


def verify_directory(nPath):
    """ If the directory does not exist, then create it, Else do nothing """
    if os.path.isdir(nPath):
        print(nPath, "exists! No action.")
    else:
        try:
            os.mkdir(nPath)
        except OSError:
            print("Creation of the directory %s failed" % nPath)
        else:
            print("Successfully created the directory %s " % nPath)


def mkdir_of_the_day(parentPath):
    """ Create a directory under 'parentPath' named after the calendar day: YYYY-MM-DD """
    desDir = None
    verify_directory(parentPath)
    try:
        desDir = os.path.join(parentPath, dayTimeStamp())
        if not os.path.exists(desDir):
            os.mkdir(desDir)
            print("Successfully created the directory %s " % desDir)
        else:
            print("Path '%s' already exists" % desDir)
    except OSError:
        print("Creation of the directory %s failed" % desDir)
    return desDir


def tcp_test(self):
    """ Test comm issues """
    return self.get_tcp_pose()


class Counter:
    """ Keeps tracks of calls """

    def __init__(self):
        """ Set the counter to zero """
        self.count = 0

    def reset(self):
        """ Set the counter to zero """
        self.__init__()

    def __call__(self):
        """ Increment the counter """
        self.count += 1
        return self.count

    def __str__(self):
        """ Return the count as a string """
        return str(self.count)

    def set_count(self, i):
        """ Manually set the counter """
        self.count = int(i)

        
        
########## CONTAINER FUNCTIONS ####################################################################


def any_A_in_B(A, B):
    """ Return True if any element of A is in B, Otherwise return False """
    for a in A:
        if a in B:
            return True
    return False


def mindex(opIterable):
    """ Return the index of the minimum item """
    return opIterable.index(min(opIterable))


def merge_Adict_into_Bdict(Adict, Bdict, overWrite=1):
    for key, val in Adict.items():
        if overWrite or (key not in Bdict):
            Bdict[key] = val
            
def is_matx_list( arg ):
    """ Return true if this is a matrix or a list """
    return type( arg ) in ( np.ndarray , list )


# _ End Container _


# = Estimation Helpers =

def Nm_to_inlb(Nm):
    return Nm * 8.85


def N_to_lbF(N):
    return N * 0.225

# _ End Helpers _


# ==== Threading ======================================================================================================


##### Timing and Scheduling #####################

def get_stopwatch():
    lastTime = now()
    
    def elapsed():
        nonlocal lastTime
        dur = now() - lastTime
        lastTime = now()
        return dur
    
    return elapsed
        

class HeartRate: 
    """ Sleeps for a time such that the period between calls to sleep results in a frequency not greater than the specified 'Hz' """
    # NOTE: This fulfills a purpose similar to the rospy rate
    
    def __init__( self , Hz ):
        """ Create a rate object with a Do-Not-Exceed frequency in 'Hz' """
        self.period = 1.0 / Hz; # Set the period as the inverse of the frequency , hearbeat will not exceed 'Hz' , but can be lower
        self.last = time.time()
    
    def sleep( self ):
        """ Sleep for a time so that the frequency is not exceeded """
        elapsed = time.time() - self.last
        if elapsed < self.period:
            time.sleep( self.period - elapsed )
        self.last = time.time()



class TimerThread( threading.Thread ):
    """ Continue to do work until the thread is killed """
    
    def __init__( self , Q , workFunc , updateHz , stopToken = None ):
        """ Set up worker and queue management """
        super().__init__()
        self.Q         = Q # --- Parent Queue that holds this thread and all the commands that it will execute
        self.workF     = workFunc # Function to be executed every iteration
        self.period    = 1.0 / updateHz
        self.stopToken = stopToken
        self.killed    = False # Flag for whether anyone has asked this thread to die
        self.count     = 0 # --- DEBUG: Count how many commands were run before dying
        self._DEBUG    = 0 # --- DEBUG: flag
        
    def run( self ):
        """ Execute the work function repeatedly until asked to stop """
        # 0. Initialize the queue with one item
        self.Q.put( time.time() )
        #print( "Does the queue begin empty?" , self.Q.empty() )
        # 1. Do until you don't
        while 1:
            # 2. Fetch the next item
            item = self.Q.get()
            # 3. Check for the poison pill and stop if found
            if item == self.stopToken:
                self.killed = True
                break
            # 4. Else execute and repeat
            else:
                # 5. Get the current time, the time elapsed, and the next scheduled time
                t_dif = item - time.time()
                # 6. If we consumed the token too early, then wait
                if t_dif > 0.0:
                    sleep( t_dif )
                # 7. Calc a time in the future to consume the next work unit, and put a new token
                t_fut = time.time() + self.period
                self.Q.put( t_fut )
                # 8. Do the work
                #print( "TimerThread: About to do the work!" )
                self.workF()
                self.count += 1
                
                if self._DEBUG:  print( "Thread executed" , self.count , "times" )

class TimerQueue( Queue ):
    """ Perform work at semi-steady intervals until asked to stop """
    
    def __init__( self , workFunc , updateHz , stopToken = None ):
        """ Set up the queue """
        super().__init__()
        self.workFunc  = workFunc
        self.updateHz  = updateHz
        self.stopToken = stopToken
        self.worker    = None
        self.running   = False
        #print( "TimerQueue: has a work function" )
        
    def start( self ):
        """ Start work and run forever """
        #print( "TimerQueue: START" )
        self.worker  = TimerThread( Q = self , workFunc = self.workFunc , updateHz = self.updateHz , stopToken = self.stopToken )
        self.running = True
        self.worker.start()
        
    def stop( self ):
        """ Administer the poison pill """
        #print( "TimerQueue: STOP" )
        if ( not self.empty() ) or ( self.is_running() ):
            self.put( self.stopToken )
        self.running = False
        
    def is_running( self ):
        """ Return true if the worker is working, otherwise return false """
        try:
            return self.worker.is_alive()
        except:
            return False

        
        
########## MONITORING FUNCTIONS ###################################################################


def ping_latency_ms( hostName ):
    """ Return the milliseconds latency for one `ping` query at the bash terminal """
    # https://stackoverflow.com/a/34455969
    try:
        output = subprocess.check_output("ping -{} 1 {}".format('c', hostName), shell=True)
        return float( str( output.decode().encode('ascii',errors='ignore') ).split( '/' )[-3].split()[0] ) * 1000.0
    except Exception as e:
        print( e )
        return float('nan')


def average_CPU_temp( cpuDct, label = 'avg_temp' ):
    """ Get an average float from all the numbers in `cpuDct` """
    total = 0
    count = 0
    for k, v in cpuDct.items():
        if isinstance( v, (float, int) ):
            count += 1
            total += v
    cpuDct[ label ] = (total / count)
    return cpuDct
    

def network_thermal_health( remoteHosts = None ):
    """ Get the local host temperatures and latency to a remote host """
    _DEBUG = 0
    if remoteHosts is None:
        remoteHosts = ['192.168.0.102']
    tmpDct = sensors_temperatures()
    rtnDct = dict( tmpDct )
    for k, v in tmpDct.items():
        rtnDct[k] = v[0].current
    average_CPU_temp( rtnDct )
    rtnDct['time'] = time.time()
    rtnDct['network'] = {}
    for host in remoteHosts:
        rtnDct[ 'network' ][ host ] = {  'latency' : ping_latency_ms( host ),  }
    return rtnDct


def temp_trends( T, series, names = None ):
    # importing package
    import matplotlib.pyplot as plt

    # plot lines
    for i,S in enumerate( series ): 
        if names is not None:
            lbl = names[i]
        else:
            lbl = str(i+1)
        plt.plot( T, S, label = lbl )
    plt.legend()
    plt.show()