########## INIT ####################################################################################

##### Resources #####
# https://chase-seibert.github.io/blog/2012/11/16/python-subprocess-asynchronous-read-stdout.html

##### Imports #####
### Standard ###
import time, threading, subprocess
now = time.time
from queue import Queue
from threading import Thread
from time import sleep
from pprint import pprint
### Local ###
from interprocess import set_non_blocking, non_block_read, PBJSON_IO


##### Globals #####
command = None
running = True



########## THREADS #################################################################################

class stdioCommWorker( threading.Thread ):
    """ Handle interprocess communication via stdio """
    # Adapted from work by dano, https://stackoverflow.com/a/25904681


    def __init__( self, updateHz, inQ, outQ, proc, procStdin, procStdout, args=(), kwargs=None ):
        """ Init thread with I/O connections to the child process """
        threading.Thread.__init__( self, args = (), kwargs = None )
        self.inQ        = inQ
        self.outQ       = outQ
        self.daemon     = True
        self.proc       = proc
        self.procStdin  = procStdin
        self.procStdout = procStdout        
        self.bgn        = now()
        self.pbj        = PBJSON_IO()
        self.per        = 1.0 / updateHz
        self.lst        = now()
        set_non_blocking( procStdout )
        sleep( 0.050 )


    def run( self ):
        """ Req'd superclass driver function """
        while (self.proc.poll() is None):
            ### Input ###
            ## From Parent ##
            if not self.inQ.empty():
                msg = self.inQ.get_nowait()
                if msg:
                    self.input_cb( msg )
            
            ## From Perception ##
            inpt = non_block_read( self.procStdout )
            self.pbj.write( inpt )

            ### Output ###
            ## To Parent ##
            if self.pbj.unpack():
                msgs = self.pbj.get_all()
                for msg in msgs:
                    self.outQ.put( msg )

            ### Rate Limit ###
            elapsed = now() - self.lst
            if (elapsed < self.per):
                sleep( self.per - elapsed )
            self.lst = now()
        

    def input_cb( self, msg ):
        """ Pass message to the Perception Process """
        ### Output ###
        ## To Perception ##
        cObj = self.pbj.pack( msg )
        self.procStdin.write( cObj )
        self.procStdin.flush()



########## MAIN ####################################################################################
if __name__ == "__main__":

    # 1. Setup comm `Queue`s
    cmndQ = Queue() # Commands     to   Perception Process
    dataQ = Queue() # Segmantation from Perception Process

    # 2. Start the Perception Process
    percProc = subprocess.Popen(
        ['python3.9', 'obj_ID_server.py'],
        stdin  = subprocess.PIPE,
        stdout = subprocess.PIPE,
        # stderr = subprocess.PIPE # 2024-07-19: Keep `stderr` open for printing info from the subprocess
    )
    sleep( 0.050 )
    print( f"Process {percProc.pid} started with status: {percProc.returncode}" )
    sleep( 10 )

    # 3. Start the Communication Thread
    commThrd = stdioCommWorker( 4.0, cmndQ, dataQ, percProc, percProc.stdin, percProc.stdout )
    commThrd.daemon = True
    commThrd.start()
        
    sleep( 1 )

    for i in range( 10 ):
        if not dataQ.empty():
            dataMsgs = dataQ.get_nowait()
            if isinstance( dataMsgs, dict ):
                print( f"Segmentation:",  )
                pprint( dataMsgs )
            elif isinstance( dataMsgs, list ):            
                for i, msg in enumerate( dataMsgs ):
                    print( f"Message {i+1} of {len(dataMsgs)}:", end=' '  )
                    pprint( msg )
        sleep( 2 )

    for i in range( 3 ):
        cmndQ.put_nowait( {
            'cmnd': "SHUTDOWN",
            'data': None,
        } )

    percProc.wait()
    print( f"Process {percProc.pid} ended with status: {percProc.returncode}" )
    commThrd.join( timeout = 1 )

    print( "\n##### TEST ENDED #####\n" )


