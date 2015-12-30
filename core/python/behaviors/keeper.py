import core, memory
import pose, commands, cfgstiff, cfgpose
import mem_objects
from task import Task
from state_machine import *
import random
from memory import joint_commands


class ChangeStiff(Node):
  OneLegSoft = [0] * core.NUM_JOINTS
  OneLegSoft[core.HeadYaw] = 0.0
  OneLegSoft[core.HeadPitch] = 0.0
  OneLegSoft[core.LShoulderPitch] = 1.0
  OneLegSoft[core.LShoulderRoll] = 1.0
  OneLegSoft[core.LElbowYaw] = 1.0
  OneLegSoft[core.LElbowRoll] = 1.0
  OneLegSoft[core.LHipYawPitch] = 1.0
  OneLegSoft[core.LHipPitch] = 1.0
  OneLegSoft[core.LHipRoll] = 1.0
  OneLegSoft[core.LKneePitch] = 1.0
  OneLegSoft[core.LAnklePitch] = 1.0
  OneLegSoft[core.LAnkleRoll] = 1.0
  OneLegSoft[core.RHipYawPitch] = 1.0
  OneLegSoft[core.RHipPitch] = 1.0
  OneLegSoft[core.RHipRoll] = 1.0
  OneLegSoft[core.RKneePitch] = 1.0
  OneLegSoft[core.RAnklePitch] = 1.0
  OneLegSoft[core.RAnkleRoll] = 1.0
  OneLegSoft[core.RShoulderPitch] = 1.0
  OneLegSoft[core.RShoulderRoll] = 1.0
  OneLegSoft[core.RElbowYaw] = 1.0
  OneLegSoft[core.RElbowRoll] = 1.0

  def run(self):
    commands.setStiffness(ChangeStiff.OneLegSoft)
    if self.getTime() > 20.0:
      self.finish()

class Stand(Node):
  def run(self):
    commands.stand()
    if self.getTime() > 3.0:
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

class Stand(Node):
  def run(self):

    commands.stand()
    if self.getTime() > 3.0:
      self.finish()

class BlockLeft(Node):
  def run(self):

    UTdebug.log(15, "Blocking left")
    if self.getTime() < 1.0:
      memory.speech.say("Blocking left")

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
    if self.getTime() < 1.0:
      memory.speech.say("Blocking right")

class BlockCenter(Node):
  def run(self):
    # Block center!
    # self.setSubtask(
    #   pose.PoseSequence(
    #     cfgpose.goalieSquatPart1, 0.4,
    #     cfgpose.goalieSquatPart2, 0.2,
    #     cfgpose.goalieSquatPart2, 3.0,
    #     cfgpose.goalieSquat5, 0.2,
    #     cfgpose.goalieSquat5, 0.3,
    #     cfgpose.goalieSquatPart2, 0.3,
    #     cfgpose.goalieSquatGetup15, 0.4,
    #     cfgpose.goalieSquatGetup2, 0.6,
    #     cfgpose.goalieSquatGetup7, 0.3))

    UTdebug.log(15, "Blocking center")
    if self.getTime() < 1.0:
      memory.speech.say("Blocking center")

class NoBlock(Node):
  def run(self):
    UTdebug.log(15, "You missed the goal")

    if self.getTime() < 1.0:
      memory.speech.say("Haha")

class Blocker(Node):
  def __init__(self):
    super(Blocker, self).__init__()
    self.vel_list = [ float(0) ]

  def run(self):
    if self.getTime() < 1.0:
      memory.speech.say("Blocker!")

    ball = mem_objects.world_objects[core.WO_BALL]
    commands.setHeadPan(ball.bearing, 1.0)
    UTdebug.log(15, ".")
    # Store a list of previous velocities
    if ball.seen:
      UTdebug.log(15, "seen")
      ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
      x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
      commands.setHeadPan(x_head_turn, .5)

      # Max size is 6
      if len(self.vel_list) > 3:
        self.vel_list = self.vel_list[:-1]
      temp_list = [ ball.absVel.x ]
      self.vel_list = temp_list + self.vel_list

      y_intersect = (ball.absVel.y / ball.absVel.x) * (-600 - ball.loc.x) + ball.loc.y
      UTdebug.log(15, "y_intersect: {}".format(y_intersect))


    if ball.distance < 750:
      # TODO
      accel = (self.vel_list[-1] - self.vel_list[0]) / float(len(self.vel_list))
      # ball_loc_pred = abs(ball.absVel.x * 3)
      t = 3
      ball_loc_pred = (ball.absVel.x * t) + ((t * t) / 2 * accel)
      # if abs(ball.absVel.x * 3) > ball.distance:
      if True:
        # Find left or right or center block
        y_intersect = (ball.absVel.y / ball.absVel.x) * (-750 - ball.loc.x) + ball.loc.y

        UTdebug.log(15, "distance: {}\t ball_loc_pred: {}\t ball.absVel.x: {}".format(ball.distance, ball_loc_pred, ball.absVel.x))
        UTdebug.log(15, "accel: {}".format(accel))
        UTdebug.log(15, "Ball is close, blocking!")
        print("y_intersect: {}".format(y_intersect))

        xvel_avg = sum(self.vel_list) / len(self.vel_list)
        print('XVEL_LENGTH = {}'.format(len(self.vel_list)))
        print('XVEL_AVG = {}'.format(xvel_avg))

        min_xvel_avg = -650

        # if ball.bearing > 30 * core.DEG_T_RAD:
        if xvel_avg < min_xvel_avg:
          if 0 < y_intersect < 500:
            choice = "left"
            # elif ball.bearing < -30 * core.DEG_T_RAD:
          elif -500 < y_intersect < -0:
            choice = "right"
          else:
            choice = "left"
          self.vel_list = []
          self.postSignal(choice)
        else:
          self.vel_list = []


class Playing(LoopingStateMachine):
  def setup(self):
    # commands.stand()
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": BlockCenter(),
      "no_block": NoBlock()
    }

    standingLeftArmPose = dict()
    # standingLeftArmPose[core.LShoulderPitch] = 0
    standingLeftArmPose[core.LShoulderRoll] = 90
    # standingLeftArmPose[core.LElbowYaw] = -0
    # standingLeftArmPose[core.LElbowRoll] = -0

    standingRightArmPose = dict()
    # standingRightArmPose[core.RShoulderPitch] = 0
    standingRightArmPose[core.RShoulderRoll] = 90
    # standingRightArmPose[core.RElbowYaw] = -0
    # standingRightArmPose[core.RElbowRoll] = -0

    standingBothArmPose = dict()
    # standingBothArmPose[core.RShoulderPitch] = 1
    standingBothArmPose[core.RShoulderRoll] = 90
    # standingBothArmPose[core.RElbowYaw] = -0
    # standingBothArmPose[core.RElbowRoll] = -0
    # standingBothArmPose[core.LShoulderPitch] = 1
    standingBothArmPose[core.LShoulderRoll] = 90
    # standingBothArmPose[core.LElbowYaw] = -0
    # standingBothArmPose[core.LElbowRoll] = -0

    # for i in range(2, core.NUM_JOINTS):
    #   val = util.getPoseJoint(i, standingLeftArmPose, False)
    #   if val != None:
    #     joint_commands.setJointCommand(i, val * core.DEG_T_RAD)



    toPoseTime = 0.2
    poses = {
      "left": pose.BlockLeft(),
      "right": pose.BlockRight(),
      "center": pose.BlockLeft(),
      "no_block": pose.BlockLeft()
    }

    stand = Stand()

    self.trans(stand, C, blocker)

    for name in blocks:
      b = blocks[name]
      p = poses[name]
      self.trans(blocker, S(name), p, T(3), pose.PoseSequence(cfgpose.sittingPoseV3, 3.0), C, Stand(), C, blocker)

    self.setFinish(None) # This ensures that the last node in trans is not the final node
