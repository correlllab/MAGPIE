import sys, select
from random import random, randrange
from time import sleep

# import uselect

# spoll = uselect.poll()
# spoll.register(sys.stdin, uselect.POLLIN)

def std_print( *args ):
    sendStr = ""
    for arg in args:
        sendStr += str(arg)+' '
    # sys.stdout.write( str.encode( sendStr[:-1], 'ascii' ) )
    sys.stdout.write( sendStr[:-1] )

class Cat:
    """ Entity that responds capriciously """

    def __init__( self, ID ):
        self.ID = ID
        # print( f'{self.ID}:', "READY\n", file=sys.stdout )
        std_print( f'{self.ID}:', "READY\n" )

    def txt_cb( self, line ):
        """ Response to an incoming message """
        if str( self.ID ) in line:
            # print( f'{self.ID}:', "MEOW\n", file=sys.stdout )
            std_print( f'{self.ID}:', "MEOW\n" )
            # sys.stdout.flush()

    def run( self ):
        """ https://repolinux.wordpress.com/2012/10/09/non-blocking-read-from-stdin-in-python/ """
        # If there's input ready, do something, else do something
        # else. Note timeout is zero so select won't block at all.

        # raise ValueError( str( type( sys.stdin ) ) )

        try:
            while True:
                # line = ""
                # while (sys.stdin in select.select([sys.stdin, ], [], [], 0.010)[0]):

                #     # raise ValueError( "FOO" )
                #     # raise ValueError( sys.stdin.buffer.read(100) )
                #     # raise ValueError( sys.stdin.readline() )

                #     # data = sys.stdin.buffer.read()
                #     # line = data.decode('ascii')

                #     # raise ValueError( line )
                #     c = sys.stdin.read(1)
                #     if c != '\n': 
                #         line += c
                #     else:
                #         break

                if (sys.stdin in select.select([sys.stdin, ], [], [], 0.010)[0]):
                    line = sys.stdin.readline()
                else:
                    line = ""

                if line:
                    # raise ValueError( line )
                    if 'SHUTDOWN' in line:
                        exit(0)
                    self.txt_cb( line )
                # else:
                    # if random() <= 0.20:
                        # print( f'{self.ID}:', f"meow --to-> {randrange(10)}\n", file=sys.stdout )
                        # std_print( f'{self.ID}:', f"meow --to-> {randrange(10)}\n" )
                std_print( f'{self.ID}:', f"meow --to-> {randrange(10)}\n" )
                # sleep( random() )
                sleep( 0.050 )    
        except BrokenPipeError:
            exit(0)

# if __name__ == "__main__":
sleep(0.050)
c = Cat( int( sys.argv[1] ) )
sleep(0.050)
c.run()
exit(0)

# FIXME: DOES THE CHILD PROCESS NEED TO BE `asyncio` AS WELL?