import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

class Ready(Task):
  def run(self):
    memory.speech.say("I want ball!")
    commands.standStraight()
    if self.getTime() > 3.0:
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print("Ball Found at ({}, {})".format(ball.imageCenterX, ball.imageCenterY))



