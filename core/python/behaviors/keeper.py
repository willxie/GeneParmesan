import core
import commands 
import mem_objects
from state_machine import *


class Playing(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setHeadPan(ball.bearing, .3)
        if not ball.seen:
            return

        # Turn head to ball
        ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
        x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
        commands.setHeadPan(x_head_turn, .5)

        # Find left or right or center block
        y_intersect = (ball.absVel.y / ball.absVel.x) * (-750 - ball.loc.x) + ball.loc.y

        UTdebug.log(15, 'y intercept: {}'.format(y_intersect))

        if -500 < y_intersect < 500:
            UTdebug.log(15, 'Shot!')
        else:
            UTdebug.log(15, 'No Shot!')
