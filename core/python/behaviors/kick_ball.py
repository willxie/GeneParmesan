import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *


# Sub tasks
class Stand(Node):
  def run(self):
    commands.stand()
    if self.getTime() > 3.0:
#      memory.speech.say("Standing complete")
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

# This is assuming that we can see the ball with bottom camera
class AlignGoal(Node):
  def run(self):
    # These are max values for Gene to adjust itself
    vel_y_gain = -0.50
    vel_x_gain = 0.50
    vel_turn_gain = 1.00

    vel_y =  -0.50 # This is the tangential velocity, fix it (try -0.50)
    vel_x = 0
    vel_turn = 0

    goal_aligned = False

    # Target position of the ball in bottom camera
    # # x_desired = 360.0 / 2
    # x_desired = 120 # Ball near left foot
    # # y_desired = 240.0 / 2
    # y_desired = 200 # Ball near the bottom of the frame when looking up
    x_desired = 117.0
    y_desired = 183.0

    # Goal centered threshold
    goal_x_right_threshold = 360.0 / 2 + 30
    goal_x_left_threshold  = 360.0 / 2 - 30

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)

    # Make sure that the ball is always at the same position relative to Gene
    if ball.seen:
      if ball.fromTopCamera:
        memory.speech.say("The ball is in top frame")
      else:
        # We Calculate the deviation from the center of the frame
        # to adjust our rotation speed
        # Y determines vel_x to make sure that Gene
        # is always a fix distance from the ball
        vel_x = vel_x_gain * (y_desired - ball.imageCenterY) / (y_desired)
        # X determines the vel_turn. Since we want to always turn
        # that's why there's an offset
        # vel_turn = 0.25 - (x_desired - ball.imageCenterX) / (x_desired) * 0.25
        vel_turn = vel_turn_gain * (x_desired - ball.imageCenterX) / (x_desired)

    if goal.seen:
      if not goal.fromTopCamera:
        memory.speech.say("Why is the goal in bottom frame?")
      else:
        # Align goal toward center of the frame
        memory.speech.say("I see goal")
        # Slow down when the goal is closer to alignment (center of top frame)
        vel_y = vel_y / 2
        if (goal.imageCenterX < goal_x_right_threshold) and (goal.imageCenterX > goal_x_left_threshold):
          goal_aligned = True

    # Exit condition
    if goal_aligned:
      self.finish()

    # commands.setWalkvelocity(0, -0.50, 0.25)
    commands.setWalkVelocity(vel_x, vel_y, vel_turn)
    commands.setHeadTilt(-10)   # Tilt head up so we can see goal (default = -22)


# This is assuming that we can see the ball with bottom camera
# Align ball to left foot and prepare to kick
class PreKick(Node):
  def run(self):
    if self.getTime() < 1.0:
      memory.speech.say("Get in position!")

    # These are max values for Gene to adjust itself
    vel_y_gain = 0.5
    vel_x_gain = 0.5
    vel_turn_gain = 1.00

    vel_y =  0
    vel_x = 0
    vel_turn = 0

    # These values are used so that the velocity doesn't drop to super low
    vel_y_min = 0.05
    vel_x_min = 0.05

    ball_aligned = False

    # Target position of the ball in bottom camera
    x_desired = 117.0
    y_desired = 183.0

    # Ball centered threshold
    ball_tolerance = 10
    ball_x_left_threshold    = x_desired - ball_tolerance
    ball_x_right_threshold   = x_desired + ball_tolerance
    ball_y_top_threshold     = y_desired - ball_tolerance
    ball_y_bottom_threshold  = y_desired + ball_tolerance

    ball = memory.world_objects.getObjPtr(core.WO_BALL)

    # Make sure that the ball is always at the same position relative to Gene
    if ball.seen:
      if ball.fromTopCamera:
        # TODO maybe walk foward?
        memory.speech.say("The ball is in top frame")
      else:
        # Similar to AlignGoal, we want to move until the ball is in front of left foot
        # The 0.05 ensures that min vel is 5% of the gain
        global_offset = 0.15
        vel_x = vel_x_gain * ((y_desired - ball.imageCenterY) / (240 / 2))
        vel_y = vel_y_gain * ((x_desired - ball.imageCenterX) / (320 / 2))
        if abs(vel_x) < global_offset:
          if vel_x > 0:
            vel_x = global_offset
          if vel_x < 0:
            vel_x = -global_offset
        if abs(vel_y) < global_offset:
          if vel_y > 0:
            vel_y = global_offset
          if vel_y < 0:
            vel_y = -global_offset


        print (vel_x)
# (ball.imageCenterY < ball_y_bottom_threshold) and
        if ((ball.imageCenterY > ball_y_top_threshold) and
            (ball.imageCenterX < ball_x_right_threshold) and (ball.imageCenterX > ball_x_left_threshold)):
          print ball.imageCenterX, ball.imageCenterY
          ball_aligned = True

    # Exit condition
    if ball_aligned:
      self.finish()

    commands.setWalkVelocity(vel_x, vel_y, vel_turn)

class Kick(Node):
  def run(self):
    if self.getTime() < 1.0:
      memory.speech.say("Kick!")
    if self.getFrames() <= 3:
      memory.walk_request.noWalk()
      memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

class Off(Node):
  def run(self):
    commands.setStiffness(cfgstiff.Zero)
    if self.getTime() > 3.0:
      memory.speech.say("Off")
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

# Button behaviors
class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 3.0:
      memory.speech.say("I am ready")
      self.finish()

class Set(Task):
  def run(self):
    commands.stand()
    if self.getTime() > 3.0:
      memory.speech.say("I am ready")
      self.finish()

class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Let's score!")

    # Movements
    stand = Stand()
    sit = Sit()
    off = Off()
    align = AlignGoal()
    pre_kick = PreKick()
    kick = Kick()
    self.trans(stand, C, align, C, pre_kick, C, kick, C, sit)

    self.setFinish(None) # This ensures that the last node in trans is not the final node
