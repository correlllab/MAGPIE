# https://chase-seibert.github.io/blog/2012/11/16/python-subprocess-asynchronous-read-stdout.html

import time, subprocess
from queue import Queue
from threading import Thread
from time import sleep
now = time.time
from random import choice, randrange

from interprocess import set_non_blocking, non_block_read, PBJSON_IO

    


########## SILLY THINGS ############################################################################

def character_factory():
    return {
    "name" : choice( ["Thing 1", "Thing 2", "Foo", "Bar", "Baz", "Xur",] ),
    "attr" : {
        "int": randrange(11),
        "str": randrange(11),
        "dex": randrange(11),
        "cha": randrange(11),
    }
}


########## THREADS #################################################################################

def log_worker( stdin, stdout ):
    ''' needs to be in a thread so we can read the stdout w/o blocking '''

    set_non_blocking( stdout )
    bgn = now()
    pbj = PBJSON_IO()

    while (now()-bgn) < 5.0:
        
        sleep( 0.050 )

        ### OUTPUT ###
        try:
            cObj = pbj.pack( character_factory() )
            stdin.write( cObj )
            stdin.flush()
        except:
            pass
        
        ### INPUT ###
        inpt = non_block_read( stdout )
        pbj.write( inpt )
        if pbj.unpack():
            msgs = pbj.get_all()
            print( "++ YES DATA ++" )
            for msg in msgs:
                print( "\tProcess Said:", msg['message'] )
        else:
            print( "!! NO DATA !!" )

        

    print( "Asking the process to shutdown!" )

    for _ in range(3):
        cObj = pbj.pack( {"message" : "SHUTDOWN"} )
        stdin.write( cObj )
        stdin.flush()
        sleep( 0.005 )



########## MAIN ####################################################################################

if __name__ == '__main__':

    process = subprocess.Popen(
        ['python3.9', 'judge.py'],
        stdin  = subprocess.PIPE,
        stdout = subprocess.PIPE,
        stderr = subprocess.PIPE
    )
    sleep( 0.050 )
    print( f"Process {process.pid} started with status: {process.returncode}" )

    thread = Thread( target = log_worker, 
                     args   = [process.stdin, process.stdout,] )
    thread.daemon = True
    thread.start()

    # sleep( 0.050 )

    process.wait()
    print( f"Process {process.pid} ended with status: {process.returncode}" )
    thread.join( timeout = 1 )





