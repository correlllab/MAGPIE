# Adapted from code by Eric Appelt: https://gist.github.com/appeltel/fd3ddeeed6c330c7208502462639d2c9

########## INIT ####################################################################################

import sys, select
import asyncio
import random
from collections import namedtuple
import time
now = time.time



########## UTILITY CLASSES #########################################################################
Msg = namedtuple( 'Msg', ('origin', 'message') )

class Hub:
    """ Distributes each message to the queue of each all subscription """

    def __init__( self ):
        """ Maintain subscriptions as a set """
        self.queue = asyncio.Queue() # Input from nodes
        self.nodes = {} # ------------ All nodes on the network

    def publish( self, message ):
        """ For each queue referenced, push message """
        c = 0
        for name, node in self.nodes.items():
            if message.origin != name:
                c +=1
                node.queue.put_nowait( message )
                print( f"\t\t\tNode {name} has {node.queue.qsize()} messages!" )
        print( f"\t\tSent to {c} nodes!" )

    def process_input( self ):
        while not self.queue.empty():
            msg = self.queue.get_nowait()
            print( '\t', (msg.origin, msg.message) )
            self.publish( msg )


class Node:
    """ Both publishes and consumes """

    def __init__( self, name, hub ):
        """ Connect to `hub` and init `queue` """
        self.name  = name
        self.hub   = hub # ----------- Reference to message distributor
        self.queue = asyncio.Queue() # Personal message queue
        self.text  = ""

    def start( self ):
        """ Add own queue to the hub when this subscription begins """
        self.hub.nodes[ self.name ] = self
        return self

    def stop( self ):
        """ Remove own queue to the hub when this subscription exits """
        del hub.nodes[ self.name ]
        # print( "Node has exited!\n", type, '\n', value, '\n', traceback, '\n' )

    def send( self, msgStr ):
        """ Send message text back to the hub """
        message = Msg( self.name, msgStr )
        self.hub.queue.put_nowait( message )

    async def recv( self ):
        """ Wait for one message text from the hub """

        print( f"Requesting from queue with {self.queue.qsize()} messages!" )

        if not self.queue.empty():
            msg = await self.queue.get()
            print( f"Popped queue how has {self.queue.qsize()} messages!" )
            self.text = msg.message
            return self.text
        else:
            return ""
        # return 



########## PUBLISHER & SUBSCRIBER PROCESSES ########################################################

async def manage_shell_subprocess( name, hub, cmd ): #, stdout_cb, stderr_cb ):
    """ Kick off a subprocess and handle messages to and from that porcess """
    process = await asyncio.create_subprocess_shell( # FIXME: IF THIS HANGS, DOES EXEC NOT HANG?
        cmd,
        stdin  = asyncio.subprocess.PIPE, 
        stdout = asyncio.subprocess.PIPE
    )
    print( f"Process {name} created!, {process.returncode}" )
    prcNode = Node( name, hub )
    prcNode.start()
    msgText = ""

    # await asyncio.sleep( 0.050 )
    # await asyncio.sleep( 1.000 )

    # process.kill()
    
    # process.stdin.flush()

    while process.returncode is None:

        # process.stdin.write( str.encode( f"Hello {name}"+'\n' ) )
        # prcNode.send( Msg( name, f"Hello {(name+1)%10}"+'\n' ) )
        
        # process.stdin.write( str.encode( f"SHUTDOWN"+'\n' ) )

        # await asyncio.sleep( 0.005 )

        # prcNode.send( f"Hello {(name+1)%10}"+'\n' )

        # Wait for a message and hand it to the process
        # msg = await prcNode.recv()
        msgText = await prcNode.recv()
        # msgText = msg.message
        if len( msgText ) > 1:
            # process.stdin.write( str.encode( str(msgText)+'\n' ) )
            process.stdin.write( str.encode( msgText, 'ascii' ) )
            await process.stdin.drain()
        
        
        # Check for a message from the process
        # if len( process.stdout._buffer ):
        #     msg = await process.stdout.read()
        #     print( "MESSAGE:", msg )
        #     msgText = msg.decode( 'ascii' )
        # else:
        #     msgText = ""
        # msg = await process.stdout.read()
        
        # print( "MESSAGE:", msg )
        # msgText = msg.decode( 'ascii' )

        # if (process.stdout in select.select([process.stdout, ], [], [], 0.010)[0]):
        #     msgText = await process.stdout.readline()
        # else:
        #     msgText = ""
        
        
        # msgText = await process.stdout.readline()
        if len( msgText ) > 1:
            prcNode.send( msgText )
        # pause
        await asyncio.sleep( random.random() )


    print( f"Process {name} ended!" )
    prcNode.stop()
    # Supports waiting to be stopped after the first task is done, 
    # or after a specified timeout, allowing lower level precision of operations
    return await process.wait()


async def message_pump( hub, timeLimit = 10.0 ):
    """ Function implements a Publisher, Sends messages to the `hub` """
    # await asyncio.sleep( 0.050 )
    bgn = now()

    

    while (now()-bgn) <= timeLimit:

        # hub.publish( Msg( -1, 'HI\n' ) )

        # Report number of current subscribers
        print( f'Writer: I have {len(hub.nodes)} subscribers now, Current traffic:' )
        # Push a message with the current itertion
        hub.process_input()
        # Let other things happen
        await asyncio.sleep( random.random()/10.0 )
    # Poison Pill: Tell all subscribers to exit
    pill = Msg( -1, 'SHUTDOWN\n' )
    hub.publish( pill )
    # hub.process_input()



########## MAIN ####################################################################################

if __name__ == '__main__':
    loop  = asyncio.get_event_loop() # ---------- Init event tracker
    hub   = Hub() # ----------------------------- Init message distributor
    nodes = [manage_shell_subprocess( x, hub, f"python3.9 cat.py {x}" ) for x in range(10)] # Init 10 Subscribers
    # Start the event loop
    loop.run_until_complete(
        # Send a collection of jobs to the event loop
        asyncio.gather(
            message_pump( hub, timeLimit = 3.0 ), # Ask Publisher to run 10 iterations and connect it to the hub
            *nodes # --------- Add readers to the event loop collection
        )
    )