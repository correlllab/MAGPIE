# https://chase-seibert.github.io/blog/2012/11/16/python-subprocess-asynchronous-read-stdout.html

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

def log_worker( stdin, stdout ):
    ''' needs to be in a thread so we can read the stdout w/o blocking '''
    set_non_blocking( stdout )
    bgn = now()
    i   = 0
    while True:
        try:
            stdin.write( str.encode( str(i), 'ascii' ) )
            stdin.flush()
        except:
            pass
        # stdin.drain()
        print( "Sent:", i )
        output = non_block_read( stdout ).strip()
        if len( output ):
            print( "Process Said:", output, '\n' )
        else:
            print()
        if (now()-bgn) > 10:
            break
        i += 1
        sleep( 0.050 )


if __name__ == '__main__':

    process = subprocess.Popen(
        ['python3.9', 'bird2.py'],
        stdin  = subprocess.PIPE,
        stdout = subprocess.PIPE,
        stderr = subprocess.PIPE
    )
    sleep( 0.050 )

    thread = Thread( target = log_worker, 
                     args   = [process.stdin, process.stdout,] )
    thread.daemon = True
    thread.start()

    process.wait()
    thread.join( timeout = 1 )





