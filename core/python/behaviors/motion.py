import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("I am ready")
      self.finish()

class Playing(StateMachine):
  class Sit(Node):
    def run(self):
      pose.Sit()
      if self.getTime() > 5.0:
        memory.speech.say("Sitting complete")
        self.finish()

  class Stand(Node):
    def run(self):
      commands.stand()
      memory.speech.say("Standing complete")
      self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 5.0:
        memory.speech.say("Turned off stiffness")
        self.finish()

  class TurningHead(Node):
    def run(self):
      commands.setHeadPan(1.5)
      memory.speech.say("Three seconds!")
      self.finish()

  def setup(self):
    # memory.speech.say("Let's exercise!")
    # #sit = self.Sit()
    # sit = pose.Sit()
    # stand = self.Stand()
    # off = self.Off()
    # self.trans(stand, C, sit, C, stand, C, off)
    # ready = Ready()
    memory.speech.say("Let's exercise!")
    # sit = pose.Sit()
    stand = self.Stand()
    turning_head = self.TurningHead()
    self.trans(stand, T(5.0), turning_head, T(5.0))
