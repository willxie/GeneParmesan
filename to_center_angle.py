import math

t = math.pi / 4
x = 25
y = -25

dx = 0 - x
dy = 0 - y

angle = math.atan(dy /dx)

# Quadrant 1
if (x > 0 and y > 0):
    angle += math.pi
# Quadrant 2
if (x < 0 and y > 0):
    angle += 0
# Quadrant 3
if (x < 0 and y < 0):
    angle += 0
# Quadrant 4        
if (x > 0 and y < 0):
    angle += math.pi
    
# Take the robot's orientation into account
angle -= t    
    

print(angle)