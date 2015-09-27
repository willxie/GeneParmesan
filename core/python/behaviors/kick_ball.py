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
      
class Spin(Node):
  def run(self):
    """Keep spinning until you see the ball in either camera"""
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
        MIN_FORWARD_VELOCITY = 0.5
        
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        if not ball.seen:
          return
      
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
	vel_turn = 0.30
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
    ball_tolerance = 15
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
        vel_turn = vel_turn_gain * ((x_desired - ball.imageCenterX) / (320 / 2))
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

	# TODO please remove 
	vel_y =  0			

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
    spin = Spin()
    sit = Sit()
    off = Off()
    pursue_ball = PursueBall()
    align = AlignGoal()
    pre_kick = PreKick()
    kick = Kick()
    self.trans(stand, C, spin, C, pursue_ball, C, align, C, pre_kick, C, kick, C, sit)

    self.setFinish(None) # This ensures that the last node in trans is not the final node
