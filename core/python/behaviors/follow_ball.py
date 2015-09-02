import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

class Set(Task):
  def run(self):
    # Stand straight up because the head can't move if crouching
    commands.standStraight()

    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      memory.speech.say("I see the ball!")
      print('Ball seen!!!!')
      ball_x, ball_y = ball.imageCenterX, ball.imageCenterY
      print('Ball seen at ({},{})'.format(ball_x, ball_y))

      x_head_turn = -(ball_x-(320.0 / 2.0)) / 160.0
      print('X HEAD TURN: {}'.format(x_head_turn))
      commands.setHeadPan(x_head_turn)

      y_head_tilt = -(ball_y-(240.0 / 2.0)) / 120.0 * 30
      print('Y HEAD TILT: {}'.format(y_head_tilt))
      commands.setHeadTilt(y_head_tilt)
    else:
      memory.speech.say("Where's the ball!?")
