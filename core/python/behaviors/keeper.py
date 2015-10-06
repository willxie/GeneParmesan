import core, memory
import pose, commands, cfgstiff
import mem_objects
from task import Task
from state_machine import *
import random

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")

class Blocker(Node):
  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    commands.setHeadPan(ball.bearing, 0.1)
    UTdebug.log(15, "bearing: {}".format(ball.bearing * 360 / (2 * 3.14)))

    if ball.distance < 500:
      ball_loc_pred = abs(ball.absVel.x * 3)
      if abs(ball.absVel.x * 3) > ball.distance:
        UTdebug.log(15, "Score~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
      else:
        UTdebug.log(15, "NOOoooooooooooooo000000000ooooooooo")
      UTdebug.log(15, "distance: {}\t ball_loc_pred: {}\t ball.absVel.x: {}".format(ball.distance, ball_loc_pred, ball.absVel.x))
      UTdebug.log(15, "Ball is close, blocking!")

      if ball.bearing > 30 * core.DEG_T_RAD:
        choice = "left"
      elif ball.bearing < -30 * core.DEG_T_RAD:
        choice = "right"
      else:
        choice = "center"
      self.postSignal(choice)

class Playing(LoopingStateMachine):
  def setup(self):
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": BlockCenter()
    }
    for name in blocks:
      b = blocks[name]
      self.trans(blocker, S(name), b, T(5), blocker)
