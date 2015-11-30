import pickle

import numpy as np

import json

import core
import commands 
import mem_objects
from state_machine import *

from softmax import Softmax


SIM_WIDTH = 3000.
SIM_HEIGHT = 2000.
SIM_GOALIE = (-750.,0.)

CLF_WIDTH = 4.
CLF_HEIGHT = 6.
CLF_GOALIE = (2.,0.)

# Load whichever model we've pickled
model = pickle.load(open('model.p', 'rb'))
clf, features, WINDOW_SIZE = model['clf'], model['features'], model['window_size']

def transform_p(x, y):
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

def transform_v(x, y):
    """Take a velocity in sim space and transform it to clf space
    
    All we have to do is divide by the sim space dimensions and multiply by the
    clf dimensions
    
    """
    return -(x/SIM_WIDTH)*CLF_WIDTH, (y/SIM_HEIGHT)*CLF_HEIGHT 


class DistanceBearingBlocker(Node):
    BALL_NOT_SEEN_THRESHOLD = 5

    def __init__(self):
        super(DistanceBearingBlocker, self).__init__()
        self.num_ball_not_seen = 0
        self.xs, self.ys = [], []

    def run(self):
        pass

class PositionBlocker(Node):
    BALL_NOT_SEEN_THRESHOLD = 5

    def __init__(self):
        super(PositionBlocker, self).__init__()
        self.num_ball_not_seen = 0
        self.xs, self.ys = [], []

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setHeadPan(ball.bearing, .3)
        if not ball.seen:
            self.num_ball_not_seen += 1

            if self.num_ball_not_seen > PositionBlocker.BALL_NOT_SEEN_THRESHOLD:
                self.num_ball_not_seen = 0
                self.xs, self.ys = [], []

            return

        # Turn head to face ball
        ball_frame_x, ball_frame_y = ball.imageCenterX, ball.imageCenterY
        x_head_turn = -(ball_frame_x-(320.0 / 2.0)) / 160.0
        commands.setHeadPan(x_head_turn, .5)

        # Read ball coordinates
        x, y = ball.loc.x, ball.loc.y
        dx, dy = ball.absVel.x, ball.absVel.y

        x_, y_ = transform_p(x, y)
        self.xs, self.ys = self.xs + [x_], self.ys + [y_]
        UTdebug.log(15, 'len(xs): {}'.format(len(self.xs)))
        if len(self.xs) < WINDOW_SIZE:
            # Not enough points to make a prediction
            UTdebug.log(15, 'Returning!')
            return

        # Need to trim a point off?
        if len(self.xs) > WINDOW_SIZE:
            self.xs, self.ys = self.xs[1:], self.ys[1:]

        UTdebug.log(15, 'Position: (x={}, y={})'.format(x, y))
        UTdebug.log(15, "Position: (x'={}, y'={})".format(*transform_p(x, y)))
        UTdebug.log(15, 'Velocity: (dx={}, dy={})'.format(dx, dy))
        UTdebug.log(15, "Velocity: (dx'={}, dy'={})".format(*transform_v(dx, dy)))

        # Predict based on the position
        X = np.array(self.xs + self.ys).reshape(2*WINDOW_SIZE, 1)
        Y = clf.predict(X)

        UTdebug.log(15, 'X: {}'.format(X))
        UTdebug.log(15, 'Y: {}'.format(Y))

class PositionVelocityBlocker(Node):
    def __init__(self):
        super(PositionVelocityBlocker, self).__init__()

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setHeadPan(ball.bearing, .3)
        if not ball.seen:
            return

        # Turn head to face ball
        ball_frame_x, ball_frame_y = ball.imageCenterX, ball.imageCenterY
        x_head_turn = -(ball_frame_x-(320.0 / 2.0)) / 160.0
        commands.setHeadPan(x_head_turn, .5)

        # Read ball coordinates
        x, y = ball.loc.x, ball.loc.y
        dx, dy = ball.absVel.x, ball.absVel.y

        # Tranform into clf space
        x_, y_, = transform_p(x, y)
        dx_, dy_, = transform_v(dx, dy)

        UTdebug.log(15, 'X: [{}, {}, {}, {}]'.format(x_, y_, dx_, dy_))
        UTdebug.log(15, "Position: (x={}, y={})".format(*transform_p(x, y)))
        UTdebug.log(15, "Velocity: (dx={}, dy={})".format(*transform_v(dx, dy)))

        # Predict based on the position
        X = np.array([x_, y_, dx_, dy_]).reshape(4, 1)
        scores, y = clf.predict(X)

        UTdebug.log(15, 'Y: {}'.format(y))
        UTdebug.log(15, 'scores: {}'.format(scores))


class Playing(LoopingStateMachine):
    def setup(self):
        blockers = {
                'position': PositionBlocker(),
                'position+velocity': PositionVelocityBlocker(),
                'distance+bearing': DistanceBearingBlocker() }

        self.trans(blockers[features])
