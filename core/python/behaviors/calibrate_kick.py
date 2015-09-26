import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *


# Sub tasks
class Stand(Node):
  def run(self):
    commands.stand()
    if self.getTime() > 1.0:
#      memory.speech.say("Standing complete")
      self.finish()

class Sit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 3.0:
      memory.speech.say("Sitting complete")
      self.finish()

class FakeSit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 0.1:
      self.finish()

class ScanLeft(Node):
  def run(self):
    commands.setHeadPan(1.0, 3)
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      commands.setHeadPan(core.joint_values[core.HeadPan], 3)
      self.postSignal("left")

    if self.getTime() > 5.0:
      self.resetTime()
      self.finish()

class ScanRight(Node):
  def run(self):
    commands.setHeadPan(-1.0, 3)
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      commands.setHeadPan(core.joint_values[core.HeadPan], 3)
      self.postSignal("right")

    if self.getTime() > 3.0:
      self.resetTime()
      self.finish()

class Off(Node):
  def run(self):
    commands.setStiffness(cfgstiff.Zero)
    if self.getTime() > 3.0:
      memory.speech.say("Off")
      self.finish()



class ChangeStiff(Node):
  left_joint = 0.0
  everything_else = 1.0
  OneLegSoft = [0] * core.NUM_JOINTS
  OneLegSoft[core.HeadYaw] = 1.0
  OneLegSoft[core.HeadPitch] = 1.0
  OneLegSoft[core.LShoulderPitch] = 0.1
  OneLegSoft[core.LShoulderRoll] = 0.1
  OneLegSoft[core.LElbowYaw] = 0.1
  OneLegSoft[core.LElbowRoll] = 0.1
  OneLegSoft[core.LHipYawPitch] = 0.0
  OneLegSoft[core.LHipPitch] = 0.0
  OneLegSoft[core.LHipRoll] = 0.0
  OneLegSoft[core.LKneePitch] = 0.0
  OneLegSoft[core.LAnklePitch] = 0.0
  OneLegSoft[core.LAnkleRoll] = 0.0
  OneLegSoft[core.RHipYawPitch] = 1.0
  OneLegSoft[core.RHipPitch] = 1.0
  OneLegSoft[core.RHipRoll] = 0.0
  OneLegSoft[core.RKneePitch] = 1.0
  OneLegSoft[core.RAnklePitch] = 1.0
  OneLegSoft[core.RAnkleRoll] = 0.0
  OneLegSoft[core.RShoulderPitch] = 0.1
  OneLegSoft[core.RShoulderRoll] = 0.1
  OneLegSoft[core.RElbowYaw] = 0.1
  OneLegSoft[core.RElbowRoll] = 0.1

  def run(self):
    commands.setStiffness(ChangeStiff.OneLegSoft)
    if self.getTime() > 20.0:
      self.finish()


class TrackBall(Node):
  def run(self):
    # Stand straight up because the head can't move if crouching

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      # memory.speech.say("I see the ball!")
      ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
#      print('Ball seen at ({},{})'.format(ball_x, ball_y))

      x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
      # print('X HEAD TURN: {}'.format(x_head_turn))
      commands.setHeadPan(x_head_turn, 1)

      # y_head_tilt = -(ball_y-(240.0 / 2.0)) / 120.0 * 30
      # # print('Y HEAD TILT: {}'.format(y_head_tilt))
      # commands.setHeadTilt(y_head_tilt)
      self.reset()

    if self.getTime() > 3.0:
      self.postSignal(self.inSignal())

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 3.0:
      memory.speech.say("I am ready")
      self.finish()


class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Let's calibrate'!")

    # Movements
    stand = Stand()
    sit = Sit()
    fake_sit = FakeSit()
    off = Off()
    change_stiff = ChangeStiff()
    self.trans(stand, C, fake_sit, C, change_stiff)
    # self.trans(stand, C, fake_sit, C, off)


    # self.setFinish(None) # This ensures that the last node in trans is not the final node
