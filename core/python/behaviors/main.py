import memory, commands
from task import Task
from state_machine import *

class Playing(Task): 
    def run(self):
        memory.speech.say("Hello World!")
        self.finish()
