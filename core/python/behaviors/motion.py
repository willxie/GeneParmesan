import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *

# Motions

class Sit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 3.0:
      memory.speech.say("Sitting complete")
      self.finish()

class Stand(Node):
  def run(self):
    commands.stand()
    if self.getTime() > 3.0:
      memory.speech.say("Standing okay complete")
      self.finish()

class Off(Node):
  def run(self):
    commands.setStiffness(cfgstiff.Zero)
    if self.getTime() > 3.0:
      memory.speech.say("Turned off stiffness")
      self.finish()

class TurningHead(Node):
  def run(self):
    commands.setHeadPan(1.0, 1)
    if self.getTime() > 5.0:
      commands.setHeadPan(-1.0, 2)
      memory.speech.say("Turn head complete!")
      self.finish()

class Spin(Node):
  def run(self):
    memory.speech.say("I am spinning")
    commands.setWalkVelocity(0, 0, -0.25)

    if self.getTime() > 5.0:
      self.finish()

class WalkForward(Node):
  def run(self):
    memory.speech.say("Walking Forward!")
    commands.setWalkVelocity(0.2, 0, 0)

    if self.getTime() > 5.0:
      self.finish()

class WalkInCurve(Node):
  def run(self):
    memory.speech.say("Walking in curve!")
    commands.setWalkVelocity(0.2, 0, 0.2)

    if self.getTime() > 5.0:
      self.finish()

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 3.0:
      memory.speech.say("I am ready")
      self.finish()


# Complex Tasks

class Set(StateMachine):
  """Sitting, Standing, and Head Turning"""
  def setup(self):
    memory.speech.say("Let's stretch!")

    # Movements
    sit = pose.Sit()
    stand_2 = Stand()
    turning_head = TurningHead()
    sit_2 = pose.Sit()
    off = Off()

    self.trans(sit, C, stand_2, C, turning_head, C, sit_2, C, off)

class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Let's do the truffle shuffle!")

    # Movements
    stand = Stand()
    walk_forward = WalkForward()
    spin = Spin()
    sit = pose.Sit()
    stand_2 = Stand()
    off = Off()

    self.trans(stand, C, walk_forward, C, spin, C, sit, C, off)

class Penalised(StateMachine):
  """Walk in a Curve"""
  def setup(self):
    memory.speech.say("Walking in a curve!")

    # Movements
    stand = Stand()
    curve_walk = WalkInCurve()
    sit = pose.Sit()
    off = Off()

    self.trans(stand, C, curve_walk, C, sit, C, off)
