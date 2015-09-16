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

    if self.getTime() > 6.0:
      commands.stand()
    if self.getTime() > 7.0:
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
  # Save a reference of the beacon_list
  def __init__(self, beacon_list):
    super(ScanLeft, self).__init__()
    self.beacon_list = beacon_list

  def run(self):
    commands.setHeadPan(1.0, 1)

    for key, value in self.beacon_list.iteritems():
      # Skip ones we have already seen
      if value:
        continue
      beacon = memory.world_objects.getObjPtr(key)
      if beacon.seen:
        commands.setHeadPan(core.joint_values[core.HeadPan], 3)
        self.beacon_list[key] = 2
        # self.beacon_list["last"] = key
        print("BEACOOOOON")
        self.postSignal("left")
        break

    if self.getTime() > 3.0:
      self.finish()

class ScanRight(Node):
  # Save a reference of the beacon_list
  def __init__(self, beacon_list):
    super(ScanRight, self).__init__()
    self.beacon_list = beacon_list

  def run(self):
    commands.setHeadPan(-1.0, 2)

    for key, value in self.beacon_list.iteritems():
      # Skip ones we have already seen
      if value:
        continue
      beacon = memory.world_objects.getObjPtr(key)
      if beacon.seen:
        commands.setHeadPan(core.joint_values[core.HeadPan], 3)
        self.beacon_list[key] = 2
        # self.beacon_list["last"] = key
        print("BEACOOOOON")
        self.postSignal("right")
        break

    print(self.getTime())
    if self.getTime() > 4.0:
      commands.setHeadPan(0, 2)
      self.finish()

class NewBeacon(Node):
  beacon_speech_list = { core.WO_BEACON_BLUE_YELLOW: "blue yellow",
                         core.WO_BEACON_YELLOW_BLUE: "yellow blue",
                         core.WO_BEACON_BLUE_PINK: "blue pink",
                         core.WO_BEACON_PINK_BLUE: "pink blue",
                         core.WO_BEACON_PINK_YELLOW: "pink yellow",
                         core.WO_BEACON_YELLOW_PINK: "yellow pink",
  }

  def __init__(self, beacon_list):
    super(NewBeacon, self).__init__()
    self.beacon_list = beacon_list

  def run(self):
    if self.getTime() < 3.0:
      # Search for most recently toggled beacon (value = 2)
      for key, value in self.beacon_list.iteritems():
        if value == 2:
          # Set the beacon to permanently seen
          self.beacon_list[key] = 1
          beacon = memory.world_objects.getObjPtr(key)
          distance = "{}".format(round(beacon.visionDistance, 1))
          memory.speech.say(NewBeacon.beacon_speech_list[key] + " " + distance)
          break
    else:
      self.postSignal(self.inSignal())


# object.visionDistance

# Complex Tasks
class Playing(StateMachine):
  """Forward Walking and Turn in Place"""
  def setup(self):
    memory.speech.say("Let's spot all the beacons!")
    beacon_list = { core.WO_BEACON_BLUE_YELLOW: 0,
                    core.WO_BEACON_YELLOW_BLUE: 0,
                    core.WO_BEACON_BLUE_PINK: 0,
                    core.WO_BEACON_PINK_BLUE: 0,
                    core.WO_BEACON_PINK_YELLOW: 0,
                    core.WO_BEACON_YELLOW_PINK: 0,
                    # "last": None
    }

    # Movements
    stand = Stand()
    spin = Spin()
    scan_left  = ScanLeft(beacon_list)
    scan_right = ScanRight(beacon_list)
    new_beacon = NewBeacon(beacon_list)

    # States
    self.trans(stand, C, spin)

    # Movements for searching for beacon
    self.trans(spin, C, scan_left)
    self.trans(scan_left, C, scan_right)
    self.trans(scan_right, C, spin)

    # When beacon is found
    self.trans(scan_left,  S("left"), new_beacon, S("left"), scan_left)
    self.trans(scan_right, S("right"), new_beacon, S("right"), scan_right)

    self.setFinish(None) # This ensures that the last node in trans is not the final node
