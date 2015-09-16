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
    commands.setHeadPan(1.0, 3)
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      commands.setHeadPan(core.joint_values[core.HeadPan], 3)
      self.postSignal("ball")

    if self.getTime() > 5.0:
      self.resetTime()
      self.finish()

class ScanRight(Node):
  def run(self):
    commands.setHeadPan(-1.0, 3)
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      commands.setHeadPan(core.joint_values[core.HeadPan], 3)
      self.postSignal("ball")

    if self.getTime() > 3.0:
      self.resetTime()
      self.finish()

class Off(Node):
  def run(self):
    commands.setStiffness(cfgstiff.Zero)
    if self.getTime() > 3.0:
      memory.speech.say("Turned off stiffness")
      self.finish()

class TrackBall(Node):
  def run(self):
    # Stand straight up because the head can't move if crouching

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      # memory.speech.say("I see the ball!")
      ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
      print('Ball seen at ({},{})'.format(ball_x, ball_y))

      x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
      # print('X HEAD TURN: {}'.format(x_head_turn))
      commands.setHeadPan(x_head_turn, 1)

      # y_head_tilt = -(ball_y-(240.0 / 2.0)) / 120.0 * 30
      # # print('Y HEAD TILT: {}'.format(y_head_tilt))
      # commands.setHeadTilt(y_head_tilt)
      self.reset()

    if self.getTime() > 3.0:
      self.finish()

class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Let's find balls'!")

    # Movements
    stand = Stand()
    sit = Sit()
    scan_left = ScanLeft()
    scan_right = ScanRight()
    track = TrackBall()
    off = Off()

    # State machine
    # One time standup
    self.trans(stand, C, scan_left)

    # Scan for ball
    self.trans(scan_left, C, scan_right)
    self.trans(scan_right, C, scan_left)

    self.trans(scan_left, S("ball"), track)
    self.trans(scan_right, S("ball"), track)

    self.trans(track, C, scan_left)

    self.setFinish(None) # This ensures that the last node in trans is not the final node
