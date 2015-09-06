import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

######################

class Ready(Task):
  def run(self):
    memory.speech.say("I want ball!")
    commands.standStraight()
    if self.getTime() > 3.0:
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print("Ball Found at ({}, {})".format(ball.imageCenterX, ball.imageCenterY))

######################


class InitHead(Node):
  '''Turn head to left most position'''
  def run(self):
    commands.standStraight()
    #commands.setHeadTilt(0)
    commands.setHeadPan(1.5, 3)
    if self.getTime() > 5.0:
      memory.speech.say("Init head complete!")      
      self.finish()

class FindGoal(Node):
  '''Get the entire goal within field of view'''
  def run(self):
    commands.setHeadPan(1.5, 10)
    goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
    solo_post = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOALPOST)
    if goal.seen:
      goal_x, goal_y = goal.imageCenterX, goal.imageCenterY
      solo_post.imageCenterX = 163
    print(solo_post.imageCenterX)
      

class Playing(StateMachine):
  """Detect goal and walk towards it"""
  def setup(self):
    memory.speech.say("Let's walk towards the goal!")
 
    # Movements
    init_head = InitHead()
    find_goal = FindGoal()

    self.trans(init_head, C, find_goal, C)
