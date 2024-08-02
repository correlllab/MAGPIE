import sys

from vispy import scene
from vispy.visuals import transforms
from vispy.color import Color
import numpy as np
canvas = scene.SceneCanvas( keys='interactive', size=(800, 600), show=True )

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#ffffff'
view.camera = 'turntable'
view.padding = 100

color = Color("#3f51b5")


cube = scene.visuals.Box( 1, 1, 1, color = color, edge_color="black")
view.add( cube )

cube.transform = transforms.STTransform( translate=(1., 2., 3.),
                                         scale=(1., 1., 1.) )

if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()