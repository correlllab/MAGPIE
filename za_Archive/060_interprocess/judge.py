import time, sys
from time import sleep
from random import random
now = time.time

from interprocess import set_non_blocking, non_block_read, PBJSON_IO

def judge_character( charSheet ):
    """ Compose one or more judgemental messages about the `charSheet` """
    msgs = []
    name = charSheet['name']
    desc = {
        "int": "smart",
        "str": "strong",
        "dex": "agile",
        "cha": "charismatic",
    }
    for key, val in charSheet['attr'].items():
        if val > 7:
            msgs.append({
                "message": f"{name} is a very {desc[key]} character!",
            })
    if not len(msgs):
        msgs.append({
            "message": f"{name} is a very BORING character!",
        })
    return msgs


# # if __name__ == "__main__":

set_non_blocking( sys.stdin )
pbj = PBJSON_IO()

sleep( 0.100 )

while True:

    # raise ValueError( f"Balderdash!" )

    ### INPUT ###
    inpt = non_block_read( sys.stdin.buffer )

    # if len( inpt ):
    #     raise ValueError( f"Got {len( inpt )} bytes!" )

    pbj.write( inpt )
    outbox = []
    if pbj.unpack():
        msgs = pbj.get_all()
        for msg in msgs:
            if ('message' in msg) and (msg['message'] == "SHUTDOWN"):
                exit(0)
            if ('attr' in msg):
                outbox.extend( judge_character( msg ) )
    else:
        outbox = [{"message": "!! NO PACKETS !!"},]

    ### OUTPUT ###
    for msg in outbox:
        # sys.stdout.write( "HI" )
        sys.stdout.buffer.write( pbj.pack( msg ) )
        sys.stdout.flush()

    ### WAIT ###
    sleep( 0.050 )
