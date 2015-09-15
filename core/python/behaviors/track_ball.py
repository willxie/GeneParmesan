import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *


# Sub tasks
class Stand(Node):
  def run(self):
    commands.stand()
    if self.getTime() > 3.0:
      memory.speech.say("Standing complete")
      self.finish()

class Sit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 3.0:
      memory.speech.say("Sitting complete")
      self.finish()

class ScanLeft(Node):
  def run(self):
    commands.setHeadPan(1.0, 4)
    # if self.getTime() > 7.0:
    #     self.resetTime()
    #     self.finish()

class ScanRight(Node):
  def run(self):
    commands.setHeadPan(-1.0, 4)
    # if self.getTime() > 7.0:
    #     self.resetTime()
    #     self.finish()

class Off(Node):
  def run(self):
    commands.setStiffness(cfgstiff.Zero)
    if self.getTime() > 3.0:
      memory.speech.say("Turned off stiffness")
      self.finish()

class Set(Task):
  def run(self):
    # Stand straight up because the head can't move if crouching
    commands.standStraight()

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      memory.speech.say("I see the ball!")
      print('Ball seen!!!!')
      ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
      print('Ball seen at ({},{})'.format(ball_x, ball_y))

      x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
      print('X HEAD TURN: {}'.format(x_head_turn))
      commands.setHeadPan(x_head_turn)

      y_head_tilt = -(ball_y-(240.0 / 2.0)) / 120.0 * 30
      print('Y HEAD TILT: {}'.format(y_head_tilt))
      commands.setHeadTilt(y_head_tilt)
    else:
      memory.speech.say("Where's the ball!?")


class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Tracking ball!")

    # Movements
    stand = Stand()
    sit = Sit()
    scan_left = ScanLeft()
    scan_right = ScanRight()
    off = Off()

    # State machine
    # self.trans(stand, C, sit, C, stand, C, off)

    self.trans(stand, C, scan_left, T(7), scan_right)
    # self.trans(scan_right, T(7), scan_left)
    # self.trans(scan_left, C, scan_right)
    # self.trans(scan_right, C, scan_left)
