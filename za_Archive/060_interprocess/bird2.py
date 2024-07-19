import sys
from time import sleep
from random import random

import os, time, fcntl, subprocess
from threading import Thread
from time import sleep
now = time.time

def set_non_blocking( output ):
    ''' even in a thread, a normal read with block until the buffer is full '''
    # Original Author, Chase Seibert: https://chase-seibert.github.io/blog/2012/11/16/python-subprocess-asynchronous-read-stdout.html
    fd = output.fileno()
    fl = fcntl.fcntl( fd, fcntl.F_GETFL )
    fcntl.fcntl( fd, fcntl.F_SETFL, fl | os.O_NONBLOCK )

def non_block_read( output ):
    """ Attempt a read from an un-blocked stream, If nothing received, then return empty string """
    # Original Author, Chase Seibert: https://chase-seibert.github.io/blog/2012/11/16/python-subprocess-asynchronous-read-stdout.html
    try:
        out = output.read()
        if out is None:
            return ''
        return out
    except:
        return ''

set_non_blocking( sys.stdin )

for _ in range(100):
    inpt = non_block_read( sys.stdin )
    if len( inpt ):
        if (int( inpt ) % 2 == 0):
            print( "Squawk!" )
            sys.stdout.flush()
        else:
            print( "Chirp!" )
            sys.stdout.flush()
    else:
        print( "NO DATA!" )
        sys.stdout.flush()
    sleep( 0.050 )

exit(0)