import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

MAX_ANGULAR_VELOCITY = 3.14/2 * 0.5

# After 1.5 meters, we don't care about how far the ball is. It doesn't make us
# approach it any faster.
DISTANCE_THRESHOLD = 1.5

# Factor to multiply thresholded distance by to get a maximum value equal to one
DISTANCE_CONSTANT = 2/3.

# Ball pursing thresholds
MAX_FORWARD_VELOCITY = .75
MIN_FORWARD_VELOCITY = 0.5

class Spin(Node):
  def run(self):
    """Keep spinning until you see the ball"""
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      self.finish()
      
    commands.setWalkVelocity(0, 0, -0.25)

class PursueBall(Node):
    ball_distances = []
    
    def run(self):
        """Approach the ball and set up for a kick
        
        If the ball is seen in the top camera, the error term in the distance of
        the ball.
        
        """
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        if not ball.seen or not ball.fromTopCamera:
            return
        
        # Ball coordinates
        ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
        
        # Calculate forward velocity
        ball_distance = ball.visionDistance / 1000
        print('Ball distance: {}'.format(ball_distance))
        ball_distance = min(ball_distance, DISTANCE_THRESHOLD)
        
        # Cache the ball distances
        PursueBall.ball_distances = (PursueBall.ball_distances + [ball_distance])[-30:]
        print('Ball distances: {}'.format(PursueBall.ball_distances))
        slope = sum(PursueBall.ball_distances[-10:])/10 - sum(PursueBall.ball_distances[:10])/10
        print('Slope: {} - {} = {}'.format(sum(PursueBall.ball_distances[-10:]) / 10,
                                           sum(PursueBall.ball_distances[:10]) / 10,
                                           slope))
        print('Input: {}'.format(1 / slope if slope else 1))
        
        
        # Get the maximum velocity to be 1
        forward_vel = ball_distance * DISTANCE_CONSTANT
        forward_vel *= MAX_FORWARD_VELOCITY
        forward_vel = max(MIN_FORWARD_VELOCITY, forward_vel)
        print('forward velocity: {}'.format(forward_vel))
        
        # Calculate sideways velocity
        angular_vel = -(ball_x-160.0) / 160.0 * MAX_ANGULAR_VELOCITY
        print('Sideways Amount: {}'.format(angular_vel))
        
        commands.setWalkVelocity(forward_vel, 0, angular_vel)
        

class Set(Task):
  def run(self):
    commands.stand()
    
# Complex Tasks
class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    # Movements
    spin = Spin()
    pursue_ball = PursueBall()

    # States
    self.trans(spin, C, pursue_ball)

    self.setFinish(None) # This ensures that the last node in trans is not the final node