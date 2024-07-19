import pbjson
from pprint import pprint

# Original Object
fName  = "magpie.png"
fName2 = "magpie2.png"
f   = open( fName, mode="rb" )
obj = {
    "name" : fName,
    "file" : f.read(),
}
f.close()

# Compressed Object
cObj = pbjson.dumps( obj )
print( type( cObj ) )

# Decompressed Object
dObj = pbjson.loads( cObj )
print( type( dObj ) )

# Copy file
f = open( fName2, mode="wb" )
f.write( dObj['file'] )
f.close()

# The file survived being packaged as bytes!




