import memory, pose, commands, cfgstiff, cfgpose, core
import mem_objects
from task import Task
from state_machine import *

beacon_list = [ core.WO_BEACON_BLUE_YELLOW,
                core.WO_BEACON_YELLOW_BLUE,
                core.WO_BEACON_BLUE_PINK,
                core.WO_BEACON_PINK_BLUE,
                core.WO_BEACON_PINK_YELLOW,
                core.WO_BEACON_YELLOW_PINK]
# Default to right = 0 (left = 1)
turn_dir = 1

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


def align_goal(vel_y):
  goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)

  # Goal centered threshold
  goal_x_right_threshold = 360.0 / 2 + 30
  goal_x_left_threshold  = 360.0 / 2 - 30

  goal_aligned = False
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
      vel_y = vel_y * (1.5/5.0)
    if (goal.imageCenterX < goal_x_right_threshold) and (goal.imageCenterX > goal_x_left_threshold):
      goal_aligned = True

  return vel_y, goal_aligned


class Dribble(Node):
  def run(self):
    vel_turn_gain = 1.00
    vel_y_gain = 1.00

    vel_y = 0.0
    vel_x = 0.30
    vel_turn = 0.05

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)

    x_desired = 360.0 / 2

    if ball.seen:
      vel_y += vel_y_gain * (x_desired - ball.imageCenterX) / (x_desired)
      vel_x = 0.4
    else:
      vel_x = -0.45

    # TODO what if the goal is not seen???
    if goal.seen:
      vel_turn += vel_turn_gain * (x_desired - goal.imageCenterX) / (x_desired)
      if goal.visionDistance < 2300:
        self.finish()

    commands.setWalkVelocity(vel_x, vel_y, vel_turn)


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
      vel_y = 0.50

    goal_aligned = False

    # Target position of the ball in bottom camera
    # # x_desired = 360.0 / 2
    # x_desired = 120 # Ball near left foot
    # # y_desired = 240.0 / 2
    # y_desired = 200 # Ball near the bottom of the frame when looking up
    x_desired = 360.0 / 2
    y_desired = 220.0

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
      vel_y, goal_aligned = align_goal(vel_y)    # Exit condition

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
    if self.getTime() < 1.0:
        memory.speech.say("Pre-kick")
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
    vel_y_min    = 0.05
    vel_x_min    = 0.25
    vel_turn_min = 0.1

    ball_aligned = False

    # Target position of the ball in bottom camera
    x_desired = 130.0
    y_desired = 225.0

    # Ball centered threshold
    ball_tolerance = 7
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
        vel_x_min = 0.25  # This is the min threshold for vel x and turn
        vel_x = vel_x_gain * ((y_desired - ball.imageCenterY) / y_desired)
        vel_turn = vel_turn_gain * ((x_desired - ball.imageCenterX) / x_desired)
        # vel_y = vel_y_gain * ((x_desired - ball.imageCenterX) / (320 / 2))
        if abs(vel_x) < vel_x_min:
          if vel_x > 0:
            vel_x = vel_x_min
          if vel_x < 0:
            vel_x = -vel_x_min
        if abs(vel_turn) < vel_turn_min:
          if vel_turn  > 0:
            vel_turn  = vel_turn_min
          if vel_turn  < 0:
            vel_turn  = -vel_turn_min

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

    # If goal not seen, do nothing for now. We are assuming that the goal is relatively aligned
    if goal.seen:
      goal_x_desired = 360.0 / 2
      # Center goal
      vel_y = vel_y_gain * (goal_x_desired - goal.imageCenterX) / (goal_x_desired)

    # Exit condition
    if ball_aligned:
      PreKick.x_errs = []
      self.finish()

    commands.setWalkVelocity(vel_x, vel_y, vel_turn)
    commands.setHeadTilt(-18)   # Tilt head up so we can see goal (default = -22)


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
####################################################################################

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


class FakeSit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 0.1:
      self.finish()

class StandStraight(Node):
  def run(self):
    commands.standStraight()
    if self.getTime() > 3.0:
      self.finish()

class Sit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 1.0:
      self.finish()

class Forward(Node):
  def run(self):
    commands.setWalkVelocity(0.4, 0, 0.05)
    if self.getTime() > 5.0:
      self.finish()

class Shuffle(Node):
  def run(self):
    commands.setWalkVelocity(0.1, 0, 0.05)
    if self.getTime() > 3.0:
      self.finish()


class BlockLeft(Node):
  def run(self):
    self.setSubtask(pose.PoseSequence(
      cfgpose.blockleft, 0.5,
      cfgpose.blockleft, 1.0,
      cfgpose.sittingPoseNoArms, 2.0,
      cfgpose.standingPose, 2.0))

    UTdebug.log(15, "Blocking left")
    memory.speech.say("Blocking left")

class BlockRight(Node):
  def run(self):
    self.setSubtask(pose.PoseSequence(
      cfgpose.blockright, 0.5,
      cfgpose.blockright, 1.0,
      cfgpose.sittingPoseNoArms, 2.0,
      cfgpose.standingPose, 2.0))

    UTdebug.log(15, "Blocking right")
    memory.speech.say("Blocking right")

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")
    if self.getTime() < 1.0:
      memory.speech.say("Blocking center")

class NoBlock(Node):
  def run(self):
    UTdebug.log(15, "You missed the goal")

    if self.getTime() < 0.5:
      self.finish()
      memory.speech.say("Haha")

class Blocker(Node):
  def __init__(self):
    super(Blocker, self).__init__()
    self.vel_list = [ float(0) ]

  def run(self):
    if self.getTime() < 1.0:
      memory.speech.say("Come at me Jake!")

    ball = mem_objects.world_objects[core.WO_BALL]

    commands.setHeadPan(ball.bearing, 1.0)
    UTdebug.log(15, ".")
    # Store a list of previous velocities
    if ball.seen:
      UTdebug.log(15, "seen")
      # print("Ball distance: {}".format(ball.distance))
      ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
      x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
      commands.setHeadPan(x_head_turn, .5)

      # if ball.bearing > 0.50:
      #   commands.setWalkVelocity(0, 0, 0.15)
      # if ball.bearing < -0.50:
      #   commands.setWalkVelocity(0, 0, -0.15)
      # else:
      #   commands.stand()

      # Max size is 6
      if len(self.vel_list) > 3:
        self.vel_list = self.vel_list[:-1]
      temp_list = [ ball.absVel.x ]
      self.vel_list = temp_list + self.vel_list

      y_intersect = (ball.absVel.y / ball.absVel.x) * (-600 - ball.loc.x) + ball.loc.y
      UTdebug.log(15, "y_intersect: {}".format(y_intersect))

      if ball.distance < 1200:
        accel = (self.vel_list[-1] - self.vel_list[0]) / float(len(self.vel_list))
        # ball_loc_pred = abs(ball.absVel.x * 3)
        t = 3
        ball_loc_pred = (ball.absVel.x * t) + ((t * t) / 2 * accel)
        # Find left or right or center block
        y_intersect = (ball.absVel.y / ball.absVel.x) * (-750 - ball.loc.x) + ball.loc.y

        UTdebug.log(15, "distance: {}\t ball_loc_pred: {}\t ball.absVel.x: {}".format(ball.distance, ball_loc_pred, ball.absVel.x))
        UTdebug.log(15, "accel: {}".format(accel))
        UTdebug.log(15, "Ball is close, blocking!")
        # print("y_intersect: {}".format(y_intersect))

        xvel_avg = sum(self.vel_list) / len(self.vel_list)
        # print('XVEL_LENGTH = {}'.format(len(self.vel_list)))
        # print('XVEL_AVG = {}'.format(xvel_avg))

        print 'DISTANCE: ', ball.distance
        min_xvel_avg = -400

        # Filter out suspicious
        if xvel_avg < -1000:
          print 'BOGUS xvel_avg: ', xvel_avg
          self.vel_list = []
          return

        # print 'xvel_avg: ', xvel_avg
        # print 'vel_list: ', self.vel_list
        # if ball.bearing > 30 * core.DEG_T_RAD:
        if xvel_avg < min_xvel_avg or ball.distance < 400:
          # print 'TRIGGERED xvel_avg: ', xvel_avg
          # print 'TRIGGERED vel_list: ', self.vel_list
          if 0 < y_intersect < 750:
            choice = "left"
            # elif ball.bearing < -30 * core.DEG_T_RAD:
          elif -750 < y_intersect <= 0:
            choice = "right"
          else:
            choice = "right"
          self.vel_list = []
          self.postSignal(choice)
        else:
          self.vel_list = []
      # else:
      #   commands.stand()

class StepBlock(Node):
  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      if ball.bearing > 0:
        commands.setWalkVelocity(0, 0.5, 0)
      else:
        commands.setWalkVelocity(0, -0.5, 0)
    else:
      commands.setWalkVelocity(0, 0.1, 0)
    if self.getTime() > 2.0:
      self.finish()

####################################################################################

# Button behaviors
class Ready(Task):
  def run(self):
    if self.getTime() < 1.0:
      memory.speech.say("I am ready")

    commands.standStraight()
    commands.setHeadTilt(-18)   # Tilt head up so we can see goal (default = -22)

    if self.getTime() > 2.0:
      commands.stand()
    #   # goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)
    #   # if goal.seen:
    #   #   print("Goal distance: {}".format(goal.visionDistance))
    #   commands.setWalkVelocity(0, 0, 0.0501 - 0.1)

    # if self.getTime() > 3.0:
      # self.finish()

class Set(LoopingStateMachine):
  def setup(self):
    # commands.stand()
    blocker = Blocker()
    blocks = {
      # "left": BlockLeft(),
      # "right": BlockRight(),
      # "center": BlockCenter(),
      # "no_block": NoBlock()
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": blocker,
      "no_block": blocker
    }

    spread_block = pose.ToPose({
      core.LHipYawPitch: -46.1407555417525,
      core.LHipRoll: -14.5924259541654,
      core.LHipPitch: -4.831634837732,
      core.LKneePitch: 63.8948760627169,
      core.LAnklePitch: -28.4793206878252,
      core.LAnkleRoll: 4.74374396349228,
      core.RHipYawPitch: -46.1407555417525,
      core.RHipRoll: -12.6540046073375,
      core.RHipPitch: -10.6373092926212,
      core.RKneePitch: 63.1086529873567,
      core.RAnklePitch: -21.7068466162991,
      core.RAnkleRoll: 0.881326629363425,
      core.LShoulderPitch: -91.4049929073173,
      core.LShoulderRoll: 8.34731078845598,
      core.LElbowYaw: -0.0902951008275686,
      core.LElbowRoll: -1.57964517010553,
      core.RShoulderPitch: -91.7613785178302,
      core.RShoulderRoll: 8.70368273859057,
      core.RElbowYaw: -0.349159270371052,
      core.RElbowRoll: -0.881326629363425}, 1)

    super_spread = pose.ToPose({
      core.LHipYawPitch: -53.7092735014381,
      core.LHipRoll: -2.8149395230157,
      core.LHipPitch: -50.1837767379148,
      core.LKneePitch: 109.071229384231,
      core.LAnklePitch: -31.1160742357736,
      core.LAnkleRoll: 3.07380369255918,
      core.RHipYawPitch: -53.7092735014381,
      core.RHipRoll: -17.2243710489382,
      core.RHipPitch: -59.3293180742919,
      core.RKneePitch: 112.328034335222,
      core.RAnklePitch: -29.9686707571032,
      core.RAnkleRoll: 12.395140437794,
      core.LShoulderPitch: -92.4596970585723,
      core.LShoulderRoll: 8.43520166269571,
      core.LElbowYaw: -1.1449992520826,
      core.LElbowRoll: -1.8433177928247,
      core.RShoulderPitch: -96.1559495505731,
      core.RShoulderRoll: 7.20952421613692,
      core.RElbowYaw: -1.22808167314663,
      core.RElbowRoll: -1.40867187480177}, 1.5)


    super_spread_2 = pose.ToPose({
      core.LHipYawPitch: -53.7092735014381,
      core.LHipRoll: -2.8149395230157,
      core.LHipPitch: -50.1837767379148,
      core.LKneePitch: 109.071229384231,
      core.LAnklePitch: -31.1160742357736,
      core.LAnkleRoll: 3.07380369255918,
      core.RHipYawPitch: -53.7092735014381,
      core.RHipRoll: -17.2243710489382,
      core.RHipPitch: -59.3293180742919,
      core.RKneePitch: 112.328034335222,
      core.RAnklePitch: -29.9686707571032,
      core.RAnkleRoll: 12.395140437794,
      core.LShoulderPitch: -92.4596970585723,
      core.LShoulderRoll: 8.43520166269571,
      core.LElbowYaw: -1.1449992520826,
      core.LElbowRoll: -1.8433177928247,
      core.RShoulderPitch: -96.1559495505731,
      core.RShoulderRoll: 7.20952421613692,
      core.RElbowYaw: -1.22808167314663,
      core.RElbowRoll: -1.40867187480177}, 3)

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
      "left": pose.ToPose(standingLeftArmPose, toPoseTime),
      "right": pose.ToPose(standingRightArmPose, toPoseTime),
      "center": pose.ToPose(standingBothArmPose, toPoseTime),
      "no_block": pose.ToPose(cfgpose.standingPose, toPoseTime)
    }

    forward = Forward()
    stand = Stand()
    stand_2 = Stand()
    shuffle = Shuffle()
    sit = Sit()

    # self.trans(stand, C, pose.ToPose(cfgpose.standingPose), C, blocker)
    self.trans(forward, C, stand, C, blocker)
    # self.trans(forward, C, stand_2, C, blocker)

    for name in blocks:
      b = blocks[name]
      p = poses[name]
      # self.trans(blocker, S(name), p, C, b, T(3), pose.ToPose(cfgpose.standingPose, toPoseTime), C, blocker)
      # self.trans(blocker, S(name), sit, t(0.5), super_spread, t(3), stand, c, shuffle, c,  blocker)
      self.trans(blocker, S(name), b, T(5.0), stand)


      # pose.PoseSequence(cfgpose.sittingPoseV3, 1), T(2)
    self.setFinish(None) # This ensures that the last node in trans is not the final node

class Playing(LoopingStateMachine):
  """Attacking behavior, try to score"""
  def setup(self):
    memory.speech.say("Charge")

    global global_dict
    # Movements
    stand = Stand()
    sit = Sit()
    off = Off()

    scan = Scan()
    spin = Spin()

    dribble = Dribble()
    pursue_ball = PursueBall()
    align = AlignGoal()
    pre_kick = PreKick()
    kick = Kick()

    # self.trans(stand, C, scan, C, pursue_ball, C,  align, C, pre_kick, C, kick, C, spin)
    self.trans(stand, C, scan, C, pursue_ball, C, align, C, dribble, C, pre_kick, C, kick, C, spin)

    # Recovery behaviors
    self.trans(spin, C, pursue_ball)
    # Lose the ball then go back to spinning
    self.trans(pursue_ball, S("restart"), spin)
    self.trans(align, S("restart"), spin)
    self.trans(pre_kick, S("restart"), spin)

    # self.setFinish(None) # This ensures that the last node in trans is not the final node
