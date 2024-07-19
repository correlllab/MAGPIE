# Adapted from code by Eric Appelt: https://gist.github.com/appeltel/fd3ddeeed6c330c7208502462639d2c9

########## INIT ####################################################################################

import asyncio
import random



########## UTILITY CLASSES #########################################################################

class Hub():
    """ Distributes each message to the queue of each all subscription """
    # FIXME: ROUTING / SORTING SHOULD TAKE PLACE HERE

    def __init__( self ):
        """ Maintain subscriptions as a set """
        self.subscriptions = set()

    def publish( self, message ):
        """ For each queue referenced, push message """
        for queue in self.subscriptions:
            queue.put_nowait( message )


class Subscription():
    """ Message subscriber connected to its message queue by reference """

    def __init__( self, hub ):
        """ Connect to `hub` and init `queue` """
        self.hub   = hub # ----------- Reference to message distributor
        self.queue = asyncio.Queue() # Personal message queue

    def __enter__( self ):
        """ Add own queue to the hub when this subscription enters _____ """
        # FIXME: WHEN IS THIS CALLED?
        self.hub.subscriptions.add( self.queue )
        return self.queue

    def __exit__( self, type, value, traceback ):
        """ Remove own queue to the hub when this subscription exits """
        self.hub.subscriptions.remove( self.queue )



########## PUBLISHER & SUBSCRIBER PROCESSES ########################################################

async def reader( name, hub ):
    """ Function implements a Subscriber, Manages a `Subscription` """

    # Wait some random amount of time before beginning ...
    await asyncio.sleep( random.random() * 15 )
    print( f'Reader {name} has decided to subscribe now!' )

    ## Subscriber ##
    msg = ""
    # Init queue object and connect it to the `hub` object
    with Subscription( hub ) as queue:
        # While the subscriber has not been sent the poison pill
        while msg != 'SHUTDOWN':
            # Wait for a message and print it
            msg = await queue.get()
            print(f'Reader {name} got message: {msg}')
            # With a 10% chance, stop looking for messages
            if random.random() < 0.1:
                print(f'Reader {name} has read enough')
                break
    # Notify user that the subscriber has stopped
    print( f'Reader {name} is shutting down' )


async def writer( iterations, hub ):
    """ Function implements a Publisher, Sends messages to the `hub` """
    for x in range( iterations ):
        # Report number of current subscribers
        print( f'Writer: I have {len(hub.subscriptions)} subscribers now' )
        # Push a message with the current itertion
        hub.publish(f'Hello world - {x}')
        # Let other things happen
        await asyncio.sleep(3)
    # Poison Pill: Tell all subscribers to exit
    hub.publish('SHUTDOWN')



########## MAIN ####################################################################################
# This has the live reporting behavior that we need!

if __name__ == '__main__':
    loop    = asyncio.get_event_loop() # ---------- Init event tracker
    hub     = Hub() # ----------------------------- Init message distributor
    readers = [reader(x, hub) for x in range(10)] # Init 10 Subscribers
    # Start the event loop
    loop.run_until_complete(
        # Send a collection of jobs to the event loop
        asyncio.gather(
            writer( 10, hub ), # Ask Publisher to run 10 iterations and connect it to the hub
            *readers # --------- Add readers to the event loop collection
        )
    )