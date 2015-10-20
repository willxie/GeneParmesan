import memory, commands
from task import Task
from state_machine import *

class Playing(Task):
    def run(self):
        memory.speech.say("Walking!")
        commands.setWalkVelocity(0, 0.5, 0)
        commands.setHeadPan(1.00, 10)
        self.finish()

class Finished(Task):
    def run(self):
        memory.speech.say("Stopping!")
        commands.setWalkVelocity(0, 0, 0)
        self.finish()
