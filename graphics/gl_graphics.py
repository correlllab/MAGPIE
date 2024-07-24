import pyglet
from pyglet import gl

import numpy as np


class Quad_Model:
    """ Base class for quad-based models of a solid color """

    def __init__( self, Nvrt, Nqua ):
        """ Allocate space """
        self.Nvrt = Nvrt
        self.Nqua = Nqua
        self.colr = [255 for _ in range(3)]
        self.pose = np.eye(4)
        self.vert = [0.0 for _ in range( Nvrt * 3 )]
        self.vOut = [0.0 for _ in range( Nvrt * 3 )]
        self.indx = [0.0 for _ in range( Nqua * 4 )]
        self.btch = pyglet.graphics.Batch()

    def transform( self, xform ):
        """ Move the model """
        self.pose = xform.dot( self.pose )
        vec = np.array( [0.0, 0.0, 0.0, 1.0,] )
        for i in range( self.Nvrt ):
            bgn = i*3
            end = bgn+3
            vec[0:3] = self.vert[ bgn:end ]
            res = self.pose.dot( vec )
            self.vOut[ bgn:end ] = res[0:3]
            # self.btch.

    def draw( self ):
        gl.glColorP3ui( *self.colr )
        # pyglet.graphics.draw_indexed(

        # )


class Cuboid( Quad_Model ):

    def __init__( self, xLen, yLen, zLen ):
        self.vertices = (
            0   , 0   , 0   ,	# vertex 0
            0   , 0   , zLen,	# vertex 1
            0   , yLen, 0   ,	# vertex 2
            0   , yLen, zLen,	# vertex 3
            xLen, 0   , 0   ,	# vertex 4
            xLen, 0   , zLen,	# vertex 5
            xLen, yLen, 0   ,	# vertex 6
            xLen, yLen, zLen,	# vertex 7
        )
        self.indices = (
            0, 1, 3, 2, # top face
            4, 5, 7, 6, # bottom face
            0, 4, 6, 2, # left face
            1, 5, 7, 3, # right face
            0, 1, 5, 4, # down face
            2, 3, 7, 6, # up face
        )

    def transform( self, xform ):
        pass

    def draw( self ):
        gl.gl
        # glTranslatef(x, y, z)
        # glTranslated( x , y , z ) # This moves the origin of drawing , so that we can use the above coordinates at each draw location