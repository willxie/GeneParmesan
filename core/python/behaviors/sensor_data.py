import core
from task import Task
import commands, cfgstiff

joint_sensors = {"HeadYaw": core.HeadYaw,
 "HeadPitch": core.HeadPitch,
 "LHipYawPitch": core.LHipYawPitch,
 "LHipRoll": core.LHipRoll,
 "LHipPitch": core.LHipPitch,
 "LKneePitch": core.LKneePitch,
 "LAnklePitch": core.LAnklePitch,
 "LAnkleRoll": core.LAnkleRoll,
 "RHipYawPitch": core.RHipYawPitch,
 "RHipRoll": core.RHipRoll,
 "RHipPitch": core.RHipPitch,
 "RKneePitch": core.RKneePitch,
 "RAnklePitch": core.RAnklePitch,
 "RAnkleRoll": core.RAnkleRoll,
 "LShoulderPitch": core.LShoulderPitch,
 "LShoulderRoll": core.LShoulderRoll,
 "LElbowYaw": core.LElbowYaw,
 "LElbowRoll": core.LElbowRoll,
 "RShoulderPitch": core.RShoulderPitch,
 "RShoulderRoll": core.RShoulderRoll,
 "RElbowYaw": core.RElbowYaw,
 "RElbowRoll": core.RElbowRoll,
}

class Ready(Task):
  """Print out joint sensor readings every 5 seconds"""
  def run(self):
    commands.setStiffness(cfgstiff.Zero)

    if self.getTime() > 5.0:
      for joint, index in joint_sensors.items():
        print('{}: {}'.format(joint, core.joint_values[index]))
      
      self.resetTime()
