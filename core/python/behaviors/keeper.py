import core
import commands 
import mem_objects
from state_machine import *

SIM_WIDTH = 3000.
SIM_HEIGHT = 2000.
SIM_GOALIE = (-750.,0.)

CLF_WIDTH = 4.
CLF_HEIGHT = 6.
CLF_GOALIE = (2.,0.)

def transform(x, y):
    """Take a point in sim space and map it into clf space
    
    Note that we are dealing with SIM_HEIGHT/2 because the classifier space is
    centered around the center of the field heightwise
    
    """
    x_, y_ = x+SIM_WIDTH/2, y # shift world over so that it's completely in the +x region
    x__, y__ = x_/SIM_WIDTH, y_/(SIM_HEIGHT/2) # shrink down to 1x1 world
    x___, y___ = 1-x__, y__ # mirror the world
    x____, y____ = CLF_WIDTH*x___, (CLF_HEIGHT/2)*y___ # expand to clf dimensions

    # UTdebug.log(15, '(x={}, x_={}, x__={}, x___={}, x____={})'.format(x, x_, x__, x___, x____))

    return x____, y____


class Playing(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setHeadPan(ball.bearing, .3)
        if not ball.seen:
            return

        # Turn head to ball
        ball_frame_x, ball_frame_y = ball.imageCenterX, ball.imageCenterY
        x_head_turn = -(ball_frame_x-(320.0 / 2.0)) / 160.0
        commands.setHeadPan(x_head_turn, .5)

        # Find left or right or center block
        y_intersect = (ball.absVel.y / ball.absVel.x) * (-750 - ball.loc.x) + ball.loc.y
        x, y = ball.loc.x, ball.loc.y

        UTdebug.log(15, 'Bottom Left: (x={}, y={})'.format(*transform(-1500, -1000)))
        UTdebug.log(15, 'Top Left: (x={}, y={})'.format(*transform(-1500, 1000)))
        UTdebug.log(15, 'Top Right: (x={}, y={})'.format(*transform(1500, 1000)))
        UTdebug.log(15, 'Bottom Right: (x={}, y={})'.format(*transform(1500, -1000)))
        UTdebug.log(15, 'Goalie: (x={}, y={})'.format(*transform(*SIM_GOALIE)))

        if -500 < y_intersect < 500:
            UTdebug.log(15, 'Shot!')
        else:
            UTdebug.log(15, 'No Shot!')
