import asyncio
from asyncio import sleep
# from time import sleep
from random import random

N = 10

async def run():
# def run():
    for i in range(10):
        # await sleep( random() )
        # sleep( random() )
        print( "Bark! " )
        await sleep( random() )

run()
