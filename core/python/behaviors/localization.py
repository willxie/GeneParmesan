import math

import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

BEACON_LIST = [ core.WO_BEACON_BLUE_YELLOW,
                core.WO_BEACON_YELLOW_BLUE,
                core.WO_BEACON_BLUE_PINK,
                core.WO_BEACON_PINK_BLUE,
                core.WO_BEACON_PINK_YELLOW,
                core.WO_BEACON_YELLOW_PINK]

def visible_beacons():
  """Yields a boolean list of all beacons seen"""
  for key in BEACON_LIST:
    beacon = memory.world_objects.getObjPtr(key)
    yield beacon.seen

# Sub tasks
class Stand(Node):
  def run(self):
    commands.setHeadTilt(-10)   # Tilt head up so we can see goal (default = -22)
    commands.stand()
    if self.getTime() > 3.0:
      self.finish()

class Sit(Node):
  def run(self):
    commands.setHeadTilt(-10)   # Tilt head up so we can see goal (default = -22)
    pose.Sit()
    if self.getTime() > 5.0:
      memory.speech.say("Sitting complete")
      self.finish()

class Walk(Node):
  def run(self):
    commands.setHeadTilt(-10)   # Tilt head up so we can see goal (default = -22)
    commands.setWalkVelocity(0.4, 0, 0)

    if self.getTime() > 15.0:
      self.finish()

class Spin(Node):
  direction = 1
  LEFT, RIGHT = -1, 1
  def run(self):
    """Spin until we see at least two beacons

    While at the same time turning Cheesy's head

    """
    # Do we see at least two beacons?
    global visible_beacons
    seen_beacons = tuple(visible_beacons())
    if sum(seen_beacons) >= 2:
      self.finish()

    # Turn head in the other direction
    if self.getTime >= 3.0:
      self.resetTime()

      if Spin.direction == Spin.LEFT:
        Spin.direction = Spin.RIGHT
      else:
        Spin.direction = Spin.LEFT

    # Look left or right
    commands.setHeadPan(Spin.direction, 1)

    memory.speech.say('Spinning!')
    commands.setWalkVelocity(0, 0, 0.3)

class Turn(Node):
  def run(self):
    commands.setHeadTilt(-10)   # Tilt head up so we can see goal (default = -22)
    memory.speech.say('Turning!')
    commands.setWalkVelocity(0, 0, -0.15)

    if self.getTime() > 5.0:
      any_beacon_seen = False
      for key in BEACON_LIST:
        beacon = memory.world_objects.getObjPtr(key)
        if beacon.seen:
          any_beacon_seen = True

      if any_beacon_seen:
        self.finish()
      # else:
      #   self.postSignal("repeat")

class PursueBeacon(Node):
  beacon = None
  def run(self):
    # memory.speech.say('Pursuing beacon!')
    if self.getTime() > 2.0:
      self.finish()

    # If it's our first time through this function then record which beacon is
    # in view
    if not PursueBeacon.beacon:
      for key in BEACON_LIST:
        beacon = memory.world_objects.getObjPtr(key)
        if beacon.seen:
          PursueBeacon.beacon = beacon

    # No beacon seen? It's a trap!
    if not PursueBeacon.beacon:
      return

    self.resetTime()

    # Stop if we get too close to the beacon
    CLOSENESS_THRESHOLD = 500
    print PursueBeacon.beacon.visionDistance
    if PursueBeacon.beacon.visionDistance < CLOSENESS_THRESHOLD:
      self.finish()

    # Calculate forward velocity
    beacon_distance = PursueBeacon.beacon.visionDistance / 1000
    DISTANCE_THRESHOLD = 1.5
    beacon_distance = min(beacon_distance, DISTANCE_THRESHOLD)
    DISTANCE_CONSTANT = 2/3.
    forward_vel = beacon_distance * DISTANCE_CONSTANT
    MAX_FORWARD_VELOCITY = .50
    forward_vel *= MAX_FORWARD_VELOCITY
    MIN_FORWARD_VELOCITY = 0.35
    forward_vel = max(MIN_FORWARD_VELOCITY, forward_vel)

    # Walk towards the beacon!
    commands.setWalkVelocity(forward_vel, 0, PursueBeacon.beacon.visionBearing)

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

    vel_y =  -0.50 # This is the tangential velocity, fix it (try -0.50)
    vel_x = 0
    vel_turn = 0.30

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
  x_errs = []
  def run(self):
    if self.getTime() > 2.0:
        self.postSignal('restart')

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
    x_desired = 150.0
    y_desired = 210.0

    # Ball centered threshold
    ball_tolerance = 15
    ball_x_left_threshold    = x_desired - ball_tolerance
    ball_x_right_threshold   = x_desired + ball_tolerance
    ball_y_top_threshold     = y_desired - ball_tolerance
    ball_y_bottom_threshold  = y_desired + ball_tolerance

    ball = memory.world_objects.getObjPtr(core.WO_BALL)

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
        print(PreKick.x_errs)
        print('Integral: {}'.format(integral))
        print('(x,y) = ({},{})'.format(ball.imageCenterX, ball.imageCenterY))
        vel_x += integral

#         print('vel_x = {}'.format(vel_x))

        if ((ball.imageCenterY > ball_y_top_threshold) and (ball.imageCenterY < ball_y_bottom_threshold) and
            (ball.imageCenterX < ball_x_right_threshold) and (ball.imageCenterX > ball_x_left_threshold)):
          print ball.imageCenterX, ball.imageCenterY
          ball_aligned = True

    # Exit condition
    if ball_aligned:
      PreKick.x_errs = []
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

class LeftPan(Task):
  def run(self):
    commands.setHeadPan(1, 1)
    commands.setWalkVelocity(.3, 0, .3)

    nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)
    # print nao.loc.x, nao.loc.y, math.tan(nao.orientation)
    facing_center = abs(nao.loc.x*math.tan(nao.orientation) - nao.loc.y)
    print 'Facing center', nao.orientation, facing_center

    if self.getTime() > 3.0:
      self.finish()

class RightPan(Task):
  def run(self):
    commands.setHeadPan(-1, 1)
    commands.setWalkVelocity(.3, 0, .3)

    nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)
    # print nao.loc.x, nao.loc.y, math.tan(nao.orientation)
    facing_center = abs(nao.loc.x*math.tan(nao.orientation) - nao.loc.y)
    print 'Facing center', nao.orientation, facing_center

    if self.getTime() > 3.0:
      self.finish()


class SpinToCenter(Node):
  def run(self):
    nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)

    # Calculate the angle of the vector to origin in robot's frame
    t = nao.orientation
    x = nao.loc.x
    y = nao.loc.y

    speed = 0.4

    dx = 0 - x
    dy = 0 - y

    angle = math.atan(dy /dx)

    # Quadrant 1
    if (x > 0 and y > 0):
        angle += math.pi
    # Quadrant 2
    if (x < 0 and y > 0):
        angle += 0
    # Quadrant 3
    if (x < 0 and y < 0):
        angle += 0
    # Quadrant 4
    if (x > 0 and y < 0):
        angle += math.pi

    # Take the robot's orientation into account
    angle -= t

    vel_turn = 0.3     # Constant spin velocity
    # vel_turn = 0.0     # Constant spin velocity
    vel_y = -(speed * math.sin(angle))
    vel_x = +(speed * math.cos(angle))

    print("Robot x: {}\t y:{}\t t: {}".format(x, y, t))
    # print("vel_x: {}\t    vel_y: {}\t     angle: {}".format(vel_x, vel_y, angle))

    vel_x = 0
    vel_y = 0
    commands.setHeadTilt(0)   # Tilt head up so we can see goal (default = -22)
    commands.setWalkVelocity(vel_x, vel_y, vel_turn)

    if self.getTime() > 30.0:
      self.finish()













##############################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################








# Spin few times so that the robot get a good idea of where it is
class SpinLocalize(Node):
  def run(self):
    memory.speech.say("Spining")

    commands.setHeadPan(0, 1)

    # nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)
    # t = nao.orientation
    # x = nao.loc.x
    # y = nao.loc.y
    # print("Robot x: {}\t y:{}\t t: {}".format(x, y, t))
    # origin = memory.world_objects.getObjPtr(core.WO_TEAM_COACH)
    # print("bearing: {}".format(origin.orientation))

    beacon_list = [ core.WO_BEACON_BLUE_YELLOW,
                    core.WO_BEACON_YELLOW_BLUE,
                    core.WO_BEACON_BLUE_PINK,
                    core.WO_BEACON_PINK_BLUE,
                    core.WO_BEACON_PINK_YELLOW,
                    core.WO_BEACON_YELLOW_PINK]

    commands.setHeadTilt(-18)   # Tilt head up so we can see goal (default = -22)
    # commands.setWalkVelocity(0, 0, -0.15)

    # if self.getTime() > 5.0:
    #   any_beacon_seen = False
    #   for key in beacon_list:
    #     beacon = memory.world_objects.getObjPtr(key)
    #     if beacon.seen:
    #       any_beacon_seen = True

    vel_x = 0
    vel_y = 0
    vel_turn = 0.30
    commands.setWalkVelocity(vel_x, vel_y, vel_turn)

    if self.getTime() > 20.0:
      beacon_x_right_threshold = 360.0 / 2 + 40
      beacon_x_left_threshold  = 360.0 / 2 - 40

      any_beacon_aligned = False
      for key in beacon_list:
        beacon = memory.world_objects.getObjPtr(key)
        if beacon.seen:
          if (beacon.imageCenterX < beacon_x_right_threshold) and (beacon.imageCenterX > beacon_x_left_threshold):
            any_beacon_aligned = True

      if any_beacon_aligned:
        self.finish()




# 1 = left, -1 = right
def toCenter(self, dir):
    vel_x = 0.4
    vel_y = 0
    vel_turn = 0.05 # This is needed because forward odo is not reliable

    turn_speed = 0.3

    origin = memory.world_objects.getObjPtr(core.WO_TEAM_COACH)
    bearing = origin.orientation
    distance = origin.distance

    # Calculate forward velocity
    dist = distance / 1000.0
    DIST_THRESHOLD = 1.5
    dist = min(dist, DIST_THRESHOLD)
    DIST_CONSTANT = 2/3.
    vel_x = dist * DIST_CONSTANT
    MAX_VEL_XOCITY = 0.50
    vel_x *= MAX_VEL_XOCITY
    MIN_VEL_XOCITY = 0.35
    vel_x = max(MIN_VEL_XOCITY, vel_x)

    # Walk towards the center!
    MAX_BEARING_VELOCITY = 0.3
    vel_turn = max(-MAX_BEARING_VELOCITY , min(MAX_BEARING_VELOCITY, bearing))

    # if (bearing > 0):
    #   vel_turn += turn_speed
    # else:
    #   vel_turn -= turn_speed

    commands.setHeadTilt(-18)   # Tilt head up so we can see goal (default = -22)
    commands.setWalkVelocity(vel_x, vel_y, vel_turn)
    # commands.setHeadPan(dir * 1.5, 1)

    print("distance: {}\t\t bearing: {}".format(distance, bearing))

    # Kidnapped relocalize
    if (not memory.body_model.feet_on_ground_):
      ToCenterBoth.num_times_no_beacon = 0
      self.postSignal("spin")
      print("flying")

    if (distance < 100):
      ToCenterBoth.num_times_no_beacon = 0
      self.postSignal("spin")

# Dont use
class ToCenterLeft(Node):
  def run(self):
    if self.getTime() < 2:
      memory.speech.say("left")

    toCenter(self, 1)

# Dont use
class ToCenterRight(Node):
  def run(self):
    if self.getTime() < 2:
      memory.speech.say("right")

    toCenter(self, -1)

class ToCenterBoth(Node):
  num_times_no_beacon = 0
  def run(self):
    if any(tuple(visible_beacons())):
      ToCenterBoth.num_times_no_beacon = 0
    else:
      ToCenterBoth.num_times_no_beacon += 1

    if ToCenterBoth.num_times_no_beacon > 30 * 5:
      self.postSignal("spin")

    if (int(self.getTime()) % 4) >= 2:
      memory.speech.say("left")
      commands.setHeadPan(1.5, 1.5)
    else:
      memory.speech.say("right")
      commands.setHeadPan(-1.5, 1.5)

    toCenter(self, 0)


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
      memory.speech.say("I am set")
      self.finish()

class LeftPan(Task):
  def run(self):
    commands.setHeadPan(1, 1)

    nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)
    # print nao.loc.x, nao.loc.y, math.tan(nao.orientation)
    facing_center = abs(nao.loc.x*math.tan(nao.orientation) - nao.loc.y)
    print 'Facing center', nao.orientation, facing_center

    if self.getTime() > 3.0:
      self.finish()

class RightPan(Task):
  def run(self):
    commands.setHeadPan(-1, 1)

    nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)
    # print nao.loc.x, nao.loc.y, math.tan(nao.orientation)
    facing_center = abs(nao.loc.x*math.tan(nao.orientation) - nao.loc.y)
    print 'Facing center', nao.orientation, facing_center

    if self.getTime() > 3.0:
      self.finish()

class WalkToCenter(Task):
  direction = 1
  LEFT, RIGHT = -1, 1
  def run(self):
    # Turn head in the other direction
    if self.getTime >= 3.0:
      self.resetTime()

      if WalkToCenter.direction == WalkToCenter.LEFT:
        WalkToCenter.direction = WalkToCenter.RIGHT
      else:
        WalkToCenter.direction = WalkToCenter.LEFT

    # Look left or right
    commands.setHeadPan(WalkToCenter.direction, 1)

    # Calculate the angle of the vector to origin in robot's frame
    nao = memory.world_objects.getObjPtr(memory.robot_state.WO_SELF)
    t, x, y = nao.orientation, nao.loc.x, nao.loc.y
    speed = 0.5
    dx, dy = 0-x, 0-y
    angle = math.atan(dy / dx)

    # Quadrant 1
    if (x > 0 and y > 0):
        angle += math.pi
    # Quadrant 2
    if (x < 0 and y > 0):
        angle += 0
    # Quadrant 3
    if (x < 0 and y < 0):
        angle += 0
    # Quadrant 4
    if (x > 0 and y < 0):
        angle += math.pi

    # Take the robot's orientation into account
    angle -= t

    # Calculate forward velocity
    distance_to_center = math.sqrt(x**2 + y**2) / 1000
    DISTANCE_THRESHOLD = 1.5
    distance_to_center = min(distance_to_center, DISTANCE_THRESHOLD)
    DISTANCE_CONSTANT = 2/3.
    forward_vel = distance_to_center * DISTANCE_CONSTANT
    MAX_FORWARD_VELOCITY = .50
    forward_vel *= MAX_FORWARD_VELOCITY
    MIN_FORWARD_VELOCITY = 0.35
    forward_vel = max(MIN_FORWARD_VELOCITY, forward_vel)

    # Walk towards the center!
    print forward_vel, angle
    MAX_ANGLE_VELOCITY = .3
    turning_vel = max(-MAX_ANGLE_VELOCITY , min(MAX_ANGLE_VELOCITY, angle))
    commands.setWalkVelocity(forward_vel, 0, turning_vel)

class Set(LoopingStateMachine):
  def setup(self):
    # Movements
    left_pan = LeftPan()
    right_pan = RightPan()

    self.trans(left_pan, C, right_pan, C, left_pan)
    self.setFinish(None)

class Playing(LoopingStateMachine):
  """Forward Walking and Turn in Place"""

  def setup(self):
    memory.speech.say("Let's localize'!")

    # left = ToCenterLeft()
    # right = ToCenterRight()

    # Movements
    stand = Stand()
    sl = SpinLocalize()
    both = ToCenterBoth()

    # self.trans(stand, C, sl, C, left, C, right, C, left)
    self.trans(stand, C, sl, C, both, C, sl)
    self.trans(both, S("spin"), sl)



    # self.trans(stand, C, left, C, right, C, left)

    # self.trans(stand, C, sl, C, sl)
    # self.trans(stand, C, turn, C, pursue_beacon, C, turn)

    # spin = Spin()
    # walk_to_center = WalkToCenter()

    # self.trans(spin, C, walk_to_center)
    # self.setFinish(None) # This ensures that the last node in trans is not the final node
