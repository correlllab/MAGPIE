from pprint import pprint

from interprocess import PBJSON_IO

pbj  = PBJSON_IO()
cObj = pbj.pack( {"message" : "SHUTDOWN"} )
for _ in range(5):
    pbj.write( cObj )
pbj.unpack()

foo = bytearray()
foo.extend( bytes() )

pprint( pbj.get_all() )
print( f"There are {len(pbj.buf)} bytes left in the buffer!" )
print( len(foo) )