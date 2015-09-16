import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

# Simple tasks
class Sit(Node):
  def run(self):
    pose.Sit()
    if self.getTime() > 3.0:
      memory.speech.say("Sitting complete")
      self.finish()

class Stand(Node):
  def run(self):
    commands.stand()
    if self.getTime() > 1.5:
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
    commands.setWalkVelocity(0, 0, -0.25)

    if self.getTime() > 5.0:
      commands.stand()
    if self.getTime() > 6.0:
      self.finish()

class WalkForward(Node):
  def run(self):
    memory.speech.say("Walking Forward!")
    commands.setWalkVelocity(0.2, 0, 0)

    if self.getTime() > 10.0:
      self.finish()

class WalkInCurve(Node):
  def run(self):
    memory.speech.say("Walking in curve!")
    commands.setWalkVelocity(0.3, 0, 0.1)

    if self.getTime() > 15.0:
      self.finish()

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 3.0:
      memory.speech.say("I am ready")
      self.finish()

# Search
class ScanLeft(Node):
  def run(self):
    commands.setHeadPan(1.0, 1)
    # ball = memory.world_objects.getObjPtr(core.WO_BALL)
    # if ball.seen:
    #   commands.setHeadPan(core.joint_values[core.HeadPan], 3)

    if self.getTime() > 3.0:
      self.finish()

class ScanRight(Node):
  def run(self):
    commands.setHeadPan(-1.0, 2)
    # ball = memory.world_objects.getObjPtr(core.WO_BALL)
    # if ball.seen:
    #   commands.setHeadPan(core.joint_values[core.HeadPan], 3)

    if self.getTime() > 5.0:
      commands.setHeadPan(0, 2)
      self.finish()

# class NewBeacon(Node):
#   def __init__(self, beacon_list):
#     super(NewBeacon, self).__init__()
#     self.beacon_list = beacon_list

#   def run(self):
#     # get distance
#     beacon = memory.world_objects.getObjPtr(core.WO_BALL) ######
#     if beacon.seen:


# # Events
# class NewBeaconDetetced(Event):
#     beacon_list = [ core.WO_BEACON_BLUE_YELLOW,
#                     core.WO_BEACON_YELLOW_BLUE,
#                     core.WO_BEACON_BLUE_PINK,
#                     core.WO_BEACON_PINK_BLUE,
#                     core.WO_BEACON_PINK_YELLOW,
#                     core.WO_BEACON_YELLOW_PINK]
#     beacon_found_set = set()
#     def ready(self):
#         # Check if at least one beacon is found.
#         for beacon in beacon_list:
#             object = memory.world_objects.getObjPtr(beacon)
#             if object.seen:
#                 if beacon not in beacon_found_set:
#                     beacon_found_set.add(beacon)
#                     return True
#         return False

# Complex Tasks
class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Let's spot all the beacons!")
    beacon_list = { core.WO_BEACON_BLUE_YELLOW: False,
                    core.WO_BEACON_YELLOW_BLUE: False,
                    core.WO_BEACON_BLUE_PINK: False,
                    core.WO_BEACON_PINK_BLUE: False,
                    core.WO_BEACON_PINK_YELLOW: False,
                    core.WO_BEACON_YELLOW_PINK: False
    }
    # beacon_list = { corere.WO_BEACON_BLUE_YELLOW: "blue yellow",
    #                 core.WO_BEACON_YELLOW_BLUE: "yellow blue",
    #                 core.WO_BEACON_BLUE_PINK: "blue pink",
    #                 core.WO_BEACON_PINK_BLUE: "pink blue",
    #                 core.WO_BEACON_PINK_YELLOW: "pink yellow",
    #                 core.WO_BEACON_YELLOW_PINK: "yellow pink"
    # }


    # Movements
    stand = Stand()
    spin = Spin()
    scan_left  = ScanLeft()
    scan_right = ScanRight()
    # new_beacon = NewBeacon()

    # States
    self.trans(stand, C, spin)

    # Movements for searching for beacon
    self.trans(spin, C, scan_left)
    self.trans(scan_left, C, scan_right)
    self.trans(scan_right, C, spin)

    self.setFinish(None) # This ensures that the last node in trans is not the final node

    # When beacon is found
    # self.trans(scan_left,  S("New beacon"), new_beacon, S("return left"), scan_left)
    # self.trans(scan_right, S("New beacon"), new_beacon, S("return right"), scan_right)
