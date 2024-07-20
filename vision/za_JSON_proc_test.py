########## INIT ####################################################################################

##### Resources #####
# https://chase-seibert.github.io/blog/2012/11/16/python-subprocess-asynchronous-read-stdout.html

##### Imports #####
### Standard ###
import time, subprocess
now = time.time
from queue import Queue
from threading import Thread
from time import sleep
from pprint import pprint
### Local ###
from interprocess import stdioCommWorker




########## MAIN ####################################################################################
if __name__ == "__main__":

    # 1. Setup comm `Queue`s
    cmndQ = Queue() # Commands     to   Perception Process
    dataQ = Queue() # Segmentation from Perception Process

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


