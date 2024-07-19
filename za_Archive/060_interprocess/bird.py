import sys
from time import sleep
from random import random

for _ in range(10):
    print( "Squawk!" )
    sys.stdout.flush()
    sleep( random() )

exit(0)