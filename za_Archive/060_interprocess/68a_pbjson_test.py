import pbjson
from pprint import pprint

# Original Object
obj = {
    "name" : "thing",
    "values" : [2,4,8,16],
    "attr" : {
        "int": 10,
        "str":  6,
        "dex":  4,
        "cha":  8,
    },
}

# Compressed Object
cObj = pbjson.dumps( obj )
print( type( cObj ) )

# Wrap the Compressed Object in a packet
packet = bytearray([42,42])
packet.extend( cObj )
packet.extend( bytearray([86,86]) )
print( packet ) # Buncha bytes

# Decompressed Object
dObj = pbjson.loads( cObj )
pprint( dObj )

# Unwrap packet and Decompress, It is the same object!
payload = packet[2:-2]
dObj = pbjson.loads( payload )
pprint( dObj )
