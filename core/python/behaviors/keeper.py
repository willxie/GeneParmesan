import os
import math
import pickle

import numpy as np

import memory
import core
import pose, cfgpose
import commands 
import mem_objects
from state_machine import *

from softmax import Softmax

# Blocking poses
SITTING_BOTH_ARMS_POSE = dict()
SITTING_BOTH_ARMS_POSE[core.RShoulderRoll] = 90
SITTING_BOTH_ARMS_POSE[core.LShoulderRoll] = 90

SITTING_LEFT_ARM_POSE = dict()
SITTING_LEFT_ARM_POSE[core.LShoulderRoll] = 90

SITTING_RIGHT_ARM_POSE = dict()
SITTING_RIGHT_ARM_POSE[core.RShoulderRoll] = 90

# Blocking actions
MIDDLE_BLOCK_ACTION = pose.ToPose(SITTING_BOTH_ARMS_POSE, 1)
LEFT_BLOCK_ACTION = pose.ToPose(SITTING_LEFT_ARM_POSE, 1)
RIGHT_BLOCK_ACTION = pose.ToPose(SITTING_RIGHT_ARM_POSE, 1)

# Sitting pose
SITTING_POSE = dict()
SITTING_POSE[core.HeadYaw] = 3.68905347261559
SITTING_POSE[core.HeadPitch] = -21.8826420251569
SITTING_POSE[core.LHipYawPitch] = -5.00743024658979
SITTING_POSE[core.LHipRoll] = 0.349159270371052
SITTING_POSE[core.LHipPitch] = -46.5802099129511
SITTING_POSE[core.LKneePitch] = 122.430772042263
SITTING_POSE[core.LAnklePitch] = -69.1732052721677
SITTING_POSE[core.LAnkleRoll] = 0.173377521891604
SITTING_POSE[core.RHipYawPitch] = -5.00743024658979
SITTING_POSE[core.RHipRoll] = 0.00240422658784449
SITTING_POSE[core.RHipPitch] = -46.7608001146063
SITTING_POSE[core.RKneePitch] = 123.753957269413
SITTING_POSE[core.RAnklePitch] = -70.135196435629
SITTING_POSE[core.RAnkleRoll] = -0.0854866476518796
SITTING_POSE[core.LShoulderPitch] = -90.4381864604912
SITTING_POSE[core.LShoulderRoll] = 2.01909954130415
SITTING_POSE[core.LElbowYaw] = -0.529749472026189
SITTING_POSE[core.LElbowRoll] = -1.4917542958658
SITTING_POSE[core.RShoulderPitch] = -91.0582242031558
SITTING_POSE[core.RShoulderRoll] = 1.05710837784287
SITTING_POSE[core.RElbowYaw] = -0.437050144610776
SITTING_POSE[core.RElbowRoll] = -0.441858597786465

# Sitting action
SIT_ACTION = pose.ToPose(SITTING_POSE, 1)

# Simulator-space dimensions
SIM_WIDTH = 3000.
SIM_HEIGHT = 2000.
SIM_GOALIE = (-750.,0.)

# Classifier-space dimensions
CLF_WIDTH = 4.
CLF_HEIGHT = 6.
CLF_GOALIE = (2.,0.)

# Load whichever model we've pickled
home = os.environ['HOME']
try:
    f = open(home + '/nao/trunk/core/python/behaviors/model.py', 'rb')
except:
    f = open(home + '/python/behaviors/model.py', 'rb')

# Extract the model
model = pickle.load(f)
clf, FEATURES, WINDOW_SIZE = model['clf'], model['features'], model['window_size']
f.close()

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

def transform_d(r, theta):
    """Take a distance in sim space and translate to clf space

    - Convert to cartesian
    - Map to clf space
    - Recompute r'

    """
    x, y = r*math.cos(theta), r*math.sin(theta)
    x_, y_ = x/SIM_WIDTH, y/SIM_HEIGHT
    x__, y__ = CLF_WIDTH*x_, CLF_HEIGHT*y_

    return math.sqrt(x__**2 + y__**2)

def transform_theta(theta):
    """"Take a theta in sim space and map it to clf space

    This is the easiest transformation of all -- it is the identity mapping!

    """
    return theta

class WindowedBlocker(Node):
    """Use position or distance+bearing depending on classifier loaded in"""

    BALL_NOT_SEEN_THRESHOLD = 5

    def __init__(self):
        super(WindowedBlocker, self).__init__()
        self.num_ball_not_seen = 0
        self.x1s, self.x2s = [], []
        self.ball_bearing = 0

    def run(self):
        commands.setHeadPan(self.ball_bearing, 1.0)

        ball = mem_objects.world_objects[core.WO_BALL]
        if not ball.seen:
            self.num_ball_not_seen += 1

            if self.num_ball_not_seen > WindowedBlocker.BALL_NOT_SEEN_THRESHOLD:
                self.num_ball_not_seen = 0
                self.x1s, self.x2s = [], []

            return

        self.ball_bearing = ball.bearing

        # Read features!
        #
        if FEATURES == 'position':
            x1,  x2  = ball.loc.x, ball.loc.y
            x1_, x2_ = transform_p(x1, x2)
        elif FEATURES == 'distance+bearing':
            x1,  x2  = ball.distance, ball.bearing
            x1_, x2_ = transform_d(x1, x2), transform_theta(x2)

        self.x1s, self.x2s = self.x1s + [x1_], self.x2s + [x2_]
        UTdebug.log(15, 'len(x1s): {}'.format(len(self.x1s)))
        if len(self.x1s) < WINDOW_SIZE:
            # Not enough points to make a prediction
            return

        # Need to trim a point off?
        if len(self.x1s) > WINDOW_SIZE:
            self.x1s, self.x2s = self.x1s[1:], self.x2s[1:]

        UTdebug.log(15, 'x1: {}'.format(x1))
        UTdebug.log(15, "x2: {}".format(x2))

        # Predict based on the position
        X = np.array(self.x1s + self.x2s).reshape(2*WINDOW_SIZE, 1)
        scores, Y = clf.predict(X)

        UTdebug.log(15, 'X[0]: {}'.format(X[0]))
        UTdebug.log(15, 'Y: {}'.format(Y))
        UTdebug.log(15, 'scores: {}'.format(scores))

        # Transition to goal pose?
        if Y[0]:
            if Y[0] == 1:
                self.postSignal('right') # flipped!
            elif Y[0] == 2:
                self.postSignal('middle')
            elif Y[0] == 3:
                self.postSignal('left') # flipped!

            self.x1s, self.x2s = [], []

class MarkovBlocker(Node):
    def __init__(self):
        super(MarkovBlocker, self).__init__()
        self.ball_bearing = 0

    def run(self):
        commands.setHeadPan(self.ball_bearing, 1.0)

        ball = mem_objects.world_objects[core.WO_BALL]
        if not ball.seen:
            return

        self.ball_bearing = ball.bearing

        # Collect features!
        #
        if FEATURES == 'position+velocity':
            # Global readings
            x, y = ball.loc.x, ball.loc.y
            dx, dy = ball.absVel.x, ball.absVel.y

            # Tranform into clf space
            x_, y_, = transform_p(x, y)
            dx_, dy_, = transform_v(dx, dy)

            # Predict based on the position
            X = np.array([x_, y_, dx_, dy_]).reshape(4, 1)

        elif FEATURES == 'position':
            # Global readings
            x, y = ball.loc.x, ball.loc.y
            x_, y_, = transform_p(x, y)

            # Predict based on the position
            X = np.array([x_, y_]).reshape(2, 1)

        elif FEATURES == 'distance+bearing':
            # Global readings
            distance,  bearing  = ball.distance, ball.bearing
            distance_, bearing_ = transform_d(distance, bearing), transform_theta(bearing)

            X = np.array([distance_, bearing_]).reshape(2, 1)

        # Predict!
        scores, Y = clf.predict(X)
        if Y[0]:
            if Y[0] == 1:
                self.postSignal('right') # flipped!
            elif Y[0] == 2:
                self.postSignal('middle')
            elif Y[0] == 3:
                self.postSignal('left') # flipped!

        UTdebug.log(15, 'Y: {}'.format(Y))
        UTdebug.log(15, 'scores: {}'.format(scores))

class Playing(LoopingStateMachine):
    def setup(self):
        blocker = WindowedBlocker() if WINDOW_SIZE else MarkovBlocker()

        signals = ('left', 'middle', 'right')
        block_actions = (LEFT_BLOCK_ACTION, MIDDLE_BLOCK_ACTION, RIGHT_BLOCK_ACTION)
        for signal, block_action in zip(signals, block_actions):
            self.trans(blocker, S(signal), block_action, C, SIT_ACTION, C, blocker)

        self.setFinish(None) # This ensures that the last node in trans is not the final node
