import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

beacon_list = [ core.WO_BEACON_BLUE_YELLOW,
                core.WO_BEACON_YELLOW_BLUE,
                core.WO_BEACON_BLUE_PINK,
                core.WO_BEACON_PINK_BLUE,
                core.WO_BEACON_PINK_YELLOW,
                core.WO_BEACON_YELLOW_PINK]
# Default to right = 0 (left = 1)
turn_dir = 0

# Use the beacons to set the next turn direction
def set_turn_direction(beacon):
  global turn_dir
  # print("Beacon: {}".format(beacon))
  # print("BP: {}".format(core.WO_BEACON_BLUE_PINK))
  # print("PB: {}".format(core.WO_BEACON_PINK_BLUE))
  if beacon == core.WO_BEACON_BLUE_PINK or beacon == core.WO_BEACON_PINK_BLUE:
    turn_dir = 1
  else:
    turn_dir = 0



class Scan(Node):
  def run(self):
    memory.walk_request.noWalk()

    # Loop for beacon
    global beacon_list
    for key in beacon_list:
      beacon = memory.world_objects.getObjPtr(key)
      if beacon.seen:
        set_turn_direction(key)
        print("Changing robot direction to: {}".format(turn_dir))

    # Turn head
    if self.getTime() < 1.0:
      memory.speech.say("Scanning")
    elif self.getTime() > 0.0 and self.getTime() < 2.0:
      commands.setHeadPan(-1.0, 0.5)
    elif self.getTime() > 2.0 and self.getTime() < 3.5:
      commands.setHeadPan(1.0, 1.0)
    else:
      commands.setHeadPan(0, 0.4)

    if self.getTime() > 4.0:
      self.finish()


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

class Spin(Node):
  def run(self):
    """Keep spinning until you see the ball in either camera"""
    memory.speech.say('Spinning!')
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      self.finish()

    commands.setWalkVelocity(0, 0, -0.25)

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

class PursueBall(Node):
    ball_distances = []
    def run(self):
        """Approach the ball enough to get it into the bottom camera

        The error term we're using is the distance to the ball

        """
        MAX_ANGULAR_VELOCITY = 3.14/2 * 0.5

        # After 1.5 meters, we don't care about how far the ball is. It doesn't make us
        # approach it any faster.
        DISTANCE_THRESHOLD = 1.5

        # Factor to multiply thresholded distance by to get a maximum value equal to one
        DISTANCE_CONSTANT = 2/3.

        # Ball pursing thresholds
        MAX_FORWARD_VELOCITY = .75
        MIN_FORWARD_VELOCITY = 0.50

        if self.getTime() > 2.0:
          self.postSignal("restart")

        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        if not ball.seen:
          return

        # Reset the timer to act as a failsafe against losing the ball
        self.reset()

        # Ball in the bottom frame?
        if not ball.fromTopCamera:
          self.finish()

        # Ball coordinates
        ball_x, ball_y = ball.imageCenterX, ball.imageCenterY

        # Calculate forward velocity
        ball_distance = ball.visionDistance / 1000
#         print('Ball distance: {}'.format(ball_distance))
        ball_distance = min(ball_distance, DISTANCE_THRESHOLD)

        # Cache the ball distances
        PursueBall.ball_distances = (PursueBall.ball_distances + [ball_distance])[-30:]
#         print('Ball distances: {}'.format(PursueBall.ball_distances))
        slope = sum(PursueBall.ball_distances[-10:])/10 - sum(PursueBall.ball_distances[:10])/10
#         print('Slope: {} - {} = {}'.format(sum(PursueBall.ball_distances[-10:]) / 10,
#                                            sum(PursueBall.ball_distances[:10]) / 10,
#                                            slope))
#         print('Input: {}'.format(1 / slope if slope else 1))


        # Get the maximum velocity to be 1
        forward_vel = ball_distance * DISTANCE_CONSTANT
        forward_vel *= MAX_FORWARD_VELOCITY
        forward_vel = max(MIN_FORWARD_VELOCITY, forward_vel)
#         print('forward velocity: {}'.format(forward_vel))

        # Calculate sideways velocity
        angular_vel = -(ball_x-160.0) / 160.0 * MAX_ANGULAR_VELOCITY
#         print('Sideways Amount: {}'.format(angular_vel))

        commands.setWalkVelocity(forward_vel, 0, angular_vel)

# This is assuming that we can see the ball with bottom camera
class AlignGoal(Node):
  def run(self):
    # These are max values for Gene to adjust itself
    vel_y_gain = -0.50
    vel_x_gain = 0.50
    vel_turn_gain = 1.00


    vel_y = -0.50 # This is the tangential velocity, fix it (try -0.50 cw or left)
    vel_x = 0
    vel_turn = 0.30

    # Change turn direction based on the last seen beacon (ccw or right)
    global turn_dir
    if turn_dir:
      print("Changing robot direction to: {}".format(turn_dir))
      vel_y = 0.50

    goal_aligned = False

    # Target position of the ball in bottom camera
    # # x_desired = 360.0 / 2
    # x_desired = 120 # Ball near left foot
    # # y_desired = 240.0 / 2
    # y_desired = 200 # Ball near the bottom of the frame when looking up
    x_desired = 125.0
    y_desired = 190.0

    # Goal centered threshold
    goal_x_right_threshold = 360.0 / 2 + 30
    goal_x_left_threshold  = 360.0 / 2 - 30

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)

    # Make sure that the ball is always at the same position relative to Gene
    if ball.seen:
      if ball.fromTopCamera:
        memory.speech.say("Align goal The ball is in top frame")
        self.postSignal("restart")
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

        # Select turn directin based on where the goal is
        if (goal.imageCenterX > (360.0 / 2)):
          vel_y = abs(vel_y)
        else:
          vel_y = -abs(vel_y)

        # Slow down when the goal is closer to alignment (center of top frame)
        vel_y = vel_y * (3.0/5.0)
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
  x_errs = []
  def run(self):
    if self.getTime() > 2.0:
        self.postSignal('restart')

    # These are max values for Gene to adjust itself
    vel_y_gain = 0.5
    vel_x_gain = 0.5
    vel_turn_gain = 1.00

    vel_y =  0.15
    vel_x = 0
    vel_turn = 0

    # These values are used so that the velocity doesn't drop to super low
    vel_y_min = 0.05
    vel_x_min = 0.05

    ball_aligned = False

    # Target position of the ball in bottom camera
    x_desired = 150.0
    y_desired = 210.0

    # Ball centered threshold
    ball_tolerance = 15
    ball_x_left_threshold    = x_desired - ball_tolerance
    ball_x_right_threshold   = x_desired + ball_tolerance
    ball_y_top_threshold     = y_desired - ball_tolerance
    ball_y_bottom_threshold  = y_desired + ball_tolerance

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)

    # Make sure that the ball is always at the same position relative to Gene
    if ball.seen:
      self.reset()

      if ball.fromTopCamera:
        # TODO maybe walk foward?
        memory.speech.say("Pre kick The ball is in top frame")
        self.postSignal("restart")
      else:
        # Similar to AlignGoal, we want to move until the ball is in front of left foot
        # The 0.05 ensures that min vel is 5% of the gain
        global_offset = 0.20
        vel_x = vel_x_gain * ((y_desired - ball.imageCenterY) / y_desired)
        vel_turn = vel_turn_gain * ((x_desired - ball.imageCenterX) / x_desired)
        # vel_y = vel_y_gain * ((x_desired - ball.imageCenterX) / (320 / 2))
        if abs(vel_x) < global_offset:
          if vel_x > 0:
            vel_x = global_offset
          if vel_x < 0:
            vel_x = -global_offset
        if abs(vel_turn) < global_offset:
          if vel_turn  > 0:
            vel_turn  = global_offset
          if vel_turn  < 0:
            vel_turn  = -global_offset

        # Cache the current x velocity for ID control
        PreKick.x_errs = (PreKick.x_errs + [vel_x])[-30:]

        # Integral Control
        INTEGRAL_CONSTANT = 1 / 75.
        integral = sum(PreKick.x_errs) * INTEGRAL_CONSTANT
        # print(PreKick.x_errs)
        # print('Integral: {}'.format(integral))
        # print('(x,y) = ({},{})'.format(ball.imageCenterX, ball.imageCenterY))
        vel_x += integral

#         print('vel_x = {}'.format(vel_x))

        if ((ball.imageCenterY > ball_y_top_threshold) and (ball.imageCenterY < ball_y_bottom_threshold) and
            (ball.imageCenterX < ball_x_right_threshold) and (ball.imageCenterX > ball_x_left_threshold)):
          # print ball.imageCenterX, ball.imageCenterY
          ball_aligned = True

    if goal.seen:
      if not goal.fromTopCamera:
        memory.speech.say("Why is the goal in bottom frame?")
      else:
        # Align goal toward center of the frame
        memory.speech.say("I see goal")

        # Select turn directin based on where the goal is
        if (goal.imageCenterX > (360.0 / 2)):
          vel_y = abs(vel_y)
        else:
          vel_y = -abs(vel_y)

        # Slow down when the goal is closer to alignment (center of top frame)
        # vel_y = vel_y * (3.0/5.0)
        if (goal.imageCenterX < goal_x_right_threshold) and (goal.imageCenterX > goal_x_left_threshold):
          goal_aligned = True

    # Exit condition
    if ball_aligned:
      PreKick.x_errs = []
      self.finish()

    commands.setWalkVelocity(vel_x, vel_y, vel_turn)
    commands.setHeadTilt(-18)   # Tilt head up so we can see goal (default = -22)

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
    if self.getTime() < 1.0:
      memory.speech.say("I am ready")

    commands.standStraight()
    commands.setHeadTilt(-18)   # Tilt head up so we can see goal (default = -22)

    if self.getTime() > 3.0:
      commands.stand()
      goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)
      if goal.seen:
        print("Goal distance: {}".format(goal.visionDistance))
        # commands.setWalkVelocity(0, 0, -0.5)

    # if self.getTime() > 3.0:
      # self.finish()

class Set(Task):
  def run(self):
    if self.getTime() < 1.0:
      memory.speech.say("Goalie")

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      BEARING_THRESHOLD = .2
      LATERAL_VELOCITY = .2
      TURNING_OFFSET = .05
      if abs(ball.bearing) < BEARING_THRESHOLD:
        print 'LESSSS', ball.bearing
        commands.setWalkVelocity(0, 0, 0)
      else:
        print 'MOOOOAR', ball.bearing
        commands.setWalkVelocity(0, ball.bearing, TURNING_OFFSET)
    else:
      print 'NOOOOOWHERE'
      commands.setWalkVelocity(0, 0, 0)


class Playing(LoopingStateMachine):
  """Attacking behavior, try to score"""
  def setup(self):
    memory.speech.say("Charge")

    global global_dict
    # Movements
    stand = Stand()
    stand_2 = Stand()
    scan = Scan()
    spin = Spin()
    sit = Sit()
    off = Off()
    pursue_ball = PursueBall()
    align = AlignGoal()
    pre_kick = PreKick()
    kick = Kick()

    self.trans(stand, C, scan, C, pursue_ball, C,  align, C, pre_kick, C, kick, C, spin)

    # Recovery behaviors
    self.trans(spin, C, pursue_ball)
    # Lose the ball then go back to spinning
    self.trans(pursue_ball, S("restart"), spin)
    self.trans(align, S("restart"), spin)
    self.trans(pre_kick, S("restart"), spin)

    # self.setFinish(None) # This ensures that the last node in trans is not the final node
