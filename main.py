#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from threading import Thread
import copy
import math
import random
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

#Constants
driveRadius = 0.9 * (81/88) * (44/40)#Fudge factors and stuff
driveMultiplier = driveRadius * 2 * math.pi / 360
minAngleDeltaBeforeDriving = 10
blackThreshold = 15
whiteThreshold = 50
lineDetectionThreshold = 2#Detects rising/falling edge
lineDetectionAAThreshold = 2#Detects entire line from rising and falling edge
lineErrorThreshold = 2#how far away from a line it thinks it is and still snap
distThreshold = 2.5

startX = 9.5
startY = 10

# Create your objects here.
ev3 = EV3Brick()

gyro = GyroSensor(Port.S3)
button = TouchSensor(Port.S2)
color = ColorSensor(Port.S1)

drivetrainLeft = Motor(Port.A, Direction.COUNTERCLOCKWISE)
drivetrainRight = Motor(Port.D, Direction.COUNTERCLOCKWISE)

clawFront = Motor(Port.C)
clawBack = Motor(Port.B)

#Helper defs

class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self, target):
        return Vec2(self.x + target.x, self.y + target.y)

    def sub(self, target):
        return Vec2(self.x - target.x, self.y - target.y)

    def norm(self):
        length = math.sqrt(self.x * self.x + self.y * self.y)
        return Vec2(self.x / length, self.y / length)

    def dist(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)

    def toString(self):
        return "[" + "{:.2f}".format(self.x) + ", " + "{:.2f}".format(self.y) + "]"

    def minAADistance(self, other):
        dx = abs(self.x - other.x)
        dy = abs(self.y - other.y)
        return min(dx, dy)
        
#Go directly to target pos
class ImmediateTarget:
    def __init__(self, pos, maxDelta, reverse = False, noSlow = False, hiSpeed = False):
        self.pos = pos
        self.maxDelta = maxDelta
        self.reverse = reverse
        self.noSlow = noSlow
        self.hiSpeed = hiSpeed
    def execute(self):
        driveTowardsTarget()

#Run a target angle on a function motor. Make sure to calibrate first
class TargetAngle:
    def __init__(self, motor, target, speed):
        self.motor = motor
        self.target = target
        self.speed = speed
    def execute(self):
        global command
        self.motor.run_target(self.speed, self.target, wait=True)
        command = 0

#Run a certin motor with a certain duty cycle for a certain time
class MotorOP:
    def __init__(self, motor, dc, time):
        self.motor = motor
        self.dc = dc
        self.time = time
        self.elapsed = 0
    def execute(self):
        global command
        command.elapsed += lastDeltaTime
        command.motor.dc(command.dc)
        if(command.elapsed > command.time):
            command.motor.hold()
            command = 0
        
#Pauses for a certain amount of time. CURRENTLY DOES NOT WORK ¯\_(ツ)_/¯
class Pause:
    def __init__(self, duration):
        self.duration = duration
        self.startTime = 0
    def execute(self):
        global command
        drivetrainLeft.hold()
        drivetrainRight.hold()
        if(self.startTime == 0):
            startTime = time.time()
        if((time.time() - self.startTime) > 1000):
            command = 0
            self.startTime = 0

#Pauses until the center button is pressed
class Wait:
    def execute(self):
        global command
        drivetrainLeft.hold()
        drivetrainRight.hold()
        if(Button.CENTER in ev3.buttons.pressed()):
            command = 0

#Turns to a set angle in world space
class AlignToAngle:
    def __init__(self, angle, maxDelta):
        self.angle = angle
        self.maxDelta = maxDelta
    def execute(self):
        global command
        turningAmount = getTurningAmount(False)
        speed = turn(turningAmount, Vec2(0, 0))
        drivetrainLeft.run(speed.x)
        drivetrainRight.run(speed.y)
        if(turningAmount > -command.maxDelta and turningAmount < command.maxDelta):
            command = 0

#Runs a motor till it stops
class Calibrate:
    def __init__(self, motor, speed):
        self.motor = motor
        self.speed = speed
        self.startTime = 0
    def execute(self):
        global command
        if(self.startTime == 0):
            self.startTime = time.time()
        self.motor.dc(self.speed)
        if (time.time() - self.startTime) > 1 and abs(self.motor.speed()) < 1:
            self.motor.reset_angle(0)
            self.motor.hold()
            command = 0
            self.startTime = 0

#Maffs
#https://github.com/Team8604/2021Main/blob/649e0b92b610d750054716165ec46ad15d480a55/src/main/java/frc/robot/commands/AutonomousRotate.java#L75
#Does stuff
def clampTo360(value):
    return value-math.floor(value/360)*360

#Gets a Vec2 based off of an angle and a distance
def angleDistance(angle, distance):
    angleTemp = math.radians(angle)
    return Vec2(math.sin(angleTemp) * distance, math.cos(angleTemp) * distance)

#Gets an angle from the difference of 2 positions
def differenceAngle(basePos, measuringPos):
    deltaVector = measuringPos.sub(basePos).norm()
    return math.degrees(math.atan2(deltaVector.x, deltaVector.y))

#https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
def sqr(x):
    return x * x

def dist2(v, w):
    return sqr(v.x - w.x) + sqr(v.y - w.y)

def distToSegmentSquared(p, v, w):
    l2 = dist2(v, w)
    if(l2 == 0):
        return dist2(p, v)
    t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2
    t = max(0, min(1, t))
    x1 = v.x + t * (w.x - v.x)
    y1 = v.y + t * (w.y - v.y)
    return dist2(p, Vec2(x1, y1))

def distToSegment(p, v, w):
    return math.sqrt(distToSegmentSquared(p, v, w))

#Running the bot
#https://github.com/Team8604/2021Main/blob/649e0b92b610d750054716165ec46ad15d480a55/src/main/java/frc/robot/commands/AutonomousRotate.java#L29
#Turn PID
def getTurningAmount(reverse):
    reverseAdd = 0
    if(reverse):
        reverseAdd = 180
    targetAngle = 0
    if(isinstance(command, ImmediateTarget)):
        targetAngle = differenceAngle(position, command.pos)
    if(isinstance(command, AlignToAngle)):
        targetAngle = command.angle
    deltaAngle = clampTo360(targetAngle + reverseAdd) - gyro.angle()
    if(deltaAngle > 180):
        deltaAngle -= 360
    if(deltaAngle < -180):
        deltaAngle += 360
    if(deltaAngle > 30):
        deltaAngle = 30
    if(deltaAngle < -30):
        deltaAngle = -30
    return deltaAngle

#Drivetrain motor state
def getDrivetrainState():
    return Vec2(drivetrainLeft.angle() * driveMultiplier, drivetrainRight.angle() * driveMultiplier)

#Main code section
#Persistent state tracking
position = Vec2(startX, startY)
lastAngle = 0
lastDrivetrainState = Vec2(0, 0)
suspendDisplay = True

#Multithreaded
#Updates state tracking
def updatePosition():
    global position
    global lastAngle
    global lastDrivetrainState
    global positionFps

    cycle = 0#Yay shadowing variables
    passedTime = 1
    lastDeltaTime = 0
    currentTime = 0

    #Logic
    while True:
        currentTime = time.time()

        if(cycle % 100 == 0):
            positionFps = 100/passedTime
            passedTime = 0
        
        currentAngle = gyro.angle()
        interpolatedAngle = (currentAngle + lastAngle) / 2
        deltaAngle = currentAngle - lastAngle
        currentDrivetrainState = getDrivetrainState()
        deltaDrivetrainState = currentDrivetrainState.sub(lastDrivetrainState)
        travelledDistance = (deltaDrivetrainState.x + deltaDrivetrainState.y) / 2
        distanceVector = angleDistance(currentAngle, travelledDistance)
        position = position.add(distanceVector)

        #Update last variables
        lastAngle = currentAngle
        lastDrivetrainState = currentDrivetrainState

        cycle += 1
        lastDeltaTime = time.time() - currentTime
        passedTime += lastDeltaTime

lines = [
    [Vec2(0,31), Vec2(17,31)],
    [Vec2(17,44.5), Vec2(17,31)],
    [Vec2(38,0), Vec2(38,44.5)],
    [Vec2(38,19.5), Vec2(92,19.5)],
    [Vec2(78,19.5), Vec2(78,0)],
    [Vec2(74.5,30.5), Vec2(92,30.5)],
    [Vec2(74.5,30.5), Vec2(74.5,44.5)],
    [Vec2(55.5,19.5), Vec2(55.5, 28)],
    [Vec2(55.5, 36.5), Vec2(55.5,44.5)]]

#Checks for line stuff
#not a main loop but needs to be defined before trackLine
def reverseOffset(pos, angle):
    a1 = -90 + angle
    s1 = Vec2(pos.x + math.sin(math.radians(a1)), pos.y + math.cos(math.radians(a1)))
    s2 = Vec2(s1.x - math.sin(math.radians(angle)) * 4.75, s1.y - math.cos(math.radians(angle)) * 4.75)
    return s2

def performLineCalibration(lastRisingEdge, lastRisingEdgeAngle, lastFallingEdge, lastFallingEdgeAngle):
    global debug

    risingEdge = reverseOffset(lastRisingEdge, lastRisingEdgeAngle)
    fallingEdge = reverseOffset(lastFallingEdge, lastFallingEdgeAngle)

    if(risingEdge.dist(fallingEdge) > distThreshold):#Too far across, not useful
        return
    
    pos = Vec2((risingEdge.x + fallingEdge.x)/2, (risingEdge.y + fallingEdge.y)/2)

    #Find closest line
    closest = -1
    closestDist = 1000000000
    for x in lines:
        dist = distToSegment(pos, x[0], x[1])
        if(dist < closestDist):
            closestDist = dist
            closest = x

    if(closestDist < lineErrorThreshold):#Perform snapping
        if(closest[0].x == closest[1].x):
            positionDeltaX = pos.x - position.x
            position.x = closest[0].x + positionDeltaX
            debug += 10000 * (lines.index(closest) + 1)
        elif(closest[0].y == closest[1].y):
            positionDeltaY = pos.y - position.y
            position.y = closest[0].y + positionDeltaY
            debug += 100000 * (lines.index(closest) + 1)
    else:
        debug += 1000

reflected = 0
def trackLine():
    global reflected
    global colorFps
    global debug

    cycle = 0#Yay shadowing variables
    passedTime = 1
    lastDeltaTime = 0
    currentTime = 0

    lastBlackPos = -1#much variable jesus chrits
    lastBlackPosTime = 0
    lastBlackPosAngle = 0
    lastWhitePos = -1
    lastWhitePosTime = 0
    lastWhitePosAngle = 0
    lastFallingEdge = -1
    lastFallingEdgeAngle = 0
    lastRisingEdge = -1
    lastRisingEdgeAngle = 0

    immediateCycle = False
    while True:
        currentTime = time.time()

        if(cycle % 100 == 0):
            colorFps = 100/passedTime
            passedTime = 0

        val = color.rgb()#Get value
        reflected = (val[0] + val[1] + val[2])/3

        if(reflected < blackThreshold):#Check for colors
            lastBlackPos = position
            lastBlackPosTime = cycle
            lastBlackPosAngle = gyro.angle()
            immediateCycle = True

        if(reflected > whiteThreshold):
            lastWhitePos = position
            lastWhitePosTime = cycle
            lastWhitePosAngle = gyro.angle()
            immediateCycle = True
        
        if(cycle % 10 == 0 or immediateCycle):#Check various things
            if(isinstance(lastWhitePos, Vec2) and isinstance(lastBlackPos, Vec2)):#Rising/falling edge
                if(lastWhitePos.dist(lastBlackPos) < lineDetectionThreshold):
                    if(lastWhitePosTime > lastBlackPosTime):#Black then white
                        lastBlackPos = -1
                        lastRisingEdge = lastWhitePos
                        lastRisingEdgeAngle = lastWhitePosAngle
                        debug += 1
                    if(lastWhitePosTime < lastBlackPosTime):#White then black
                        lastWhitePos = -1
                        lastFallingEdge = lastBlackPos
                        lastFallingEdgeAngle = lastBlackPosAngle
                        debug += 10
            
            if(isinstance(lastRisingEdge, Vec2) and isinstance(lastFallingEdge, Vec2)):#Full line check
                if(lastRisingEdge.minAADistance(lastFallingEdge) < lineDetectionAAThreshold):
                    performLineCalibration(lastRisingEdge, lastRisingEdgeAngle, lastFallingEdge, lastFallingEdgeAngle)
                    debug += 100
                    lastFallingEdge = -1
                    lastRisingEdge = -1
            immediateCycle = False

        cycle += 1
        lastDeltaTime = time.time() - currentTime
        passedTime += lastDeltaTime

#Yes
def runDisplay():
    t1 = t1 = time.time()
    t2 = 0
    while True:
        if suspendDisplay:
            continue
        ev3.screen.clear()
        j = 0
        ev3.screen.draw_text(0, j*15, "POS:   " + position.toString(), Color.BLACK)
        j += 1
        ev3.screen.draw_text(0, j*15, "ROT:   " + str(gyro.angle()), Color.BLACK)
        j += 1
        ev3.screen.draw_text(0, j*15, "REFL:  " + "{:.2f}".format(reflected), Color.BLACK)
        if(isinstance(command, ImmediateTarget)):
            j += 1
            ev3.screen.draw_text(0, j*15, "DIST:  " + "{:.2f}".format(position.dist(command.pos)), Color.BLACK)
        j += 1
        ev3.screen.draw_text(0, j*15, "STACK: " + str(len(stack)), Color.BLACK)
        j += 1
        ev3.screen.draw_text(0, j*15, "FPS:   " + "{:.2f}".format(mainLoopFps) + ", ", Color.BLACK)
        j += 1
        ev3.screen.draw_text(0, j*15, "{:.2f}".format(positionFps) + ", " + "{:.2f}".format(colorFps), Color.BLACK)
        j += 1
        ev3.screen.draw_text(0, j*15, "DEBUG: " + str(debug), Color.BLACK)
        #I think this makes a 1 second cycle
        t2 = time.time()
        v1 = t2 - t1
        t1 = time.time()
        v2 = 1 - v1
        if(v2 > 0):
            time.sleep(v2)

#Target commands
#Turns by an angle, oldSpeed is a Vec2 representing the two motor speeds. This way multiple commands can be stacked.
def turn(deltaAngle, oldSpeed):
    speedAmount = deltaAngle * 10
    oldSpeed.x += speedAmount
    oldSpeed.y -= speedAmount
    return oldSpeed

#Drives a distance, oldSpeed is a Vec2 representing the two motor speeds. This way multiple commands can be stacked.
def drive(dist, oldSpeed, reverse, noSlow, hiSpeed):
    if(dist > 5):
        dist = 5
    if(noSlow):#yay dirty hacks
        dist = 5
    speedAmount = dist * 50
    if(hiSpeed):
        speedAmount *= 2
    if(reverse):
        speedAmount *= -1
    oldSpeed.x += speedAmount
    oldSpeed.y += speedAmount
    return oldSpeed

#Combines drive() and turn(), use when command is a ImmediateTarget
def driveTowardsTarget():
    global command
    if(command == 0):
        return
    speed = Vec2(0, 0)
    turningAmount = getTurningAmount(command.reverse)
    speed = turn(turningAmount, speed)
    if(turningAmount > -minAngleDeltaBeforeDriving and turningAmount < minAngleDeltaBeforeDriving):
        speed = drive(position.dist(command.pos), speed, command.reverse, command.noSlow, command.hiSpeed)
    drivetrainLeft.run(speed.x)
    drivetrainRight.run(speed.y)
    if(position.dist(command.pos) < command.maxDelta):
        command = 0

#Cycle and FPS tracking
currentTime = 0
cycle = 0
passedTime = 1
lastDeltaTime = 0
positionFps = 0
colorFps = 0
mainLoopFps = 0
i = -1
debug = 0

#Command stuff
#Command stack
test1 = [#First test stack
    ImmediateTarget(Vec2(7.5, 50), 0.5), 
    ImmediateTarget(Vec2(6.5, 29.5), 0.5), 
    ImmediateTarget(Vec2(15, 29.5), 0.5, True), 
    AlignToAngle(-90, 1),
    MotorOP(clawFront, -50, 0.25), 
    ImmediateTarget(Vec2(10.5, 29.5), 0.5), 
    ImmediateTarget(Vec2(14.5, 38.5), 0.5), 
    ImmediateTarget(Vec2(67.5, 38.5), 0.5)
]
test2 = [#Testing calibration
    Calibrate(clawBack, 20),
    Calibrate(clawFront, -50)
]
test3 = [#Testing mutliple run system
    ImmediateTarget(Vec2(9.5, 15), 0.5, False),
    ImmediateTarget(Vec2(9.5, 9.5), 0.5, True)
]
test4 = [#Testing rotation #1
    AlignToAngle(-90, 1),
    AlignToAngle(0, 1),
    AlignToAngle(90, 1),
    AlignToAngle(0, 1)
]
test5 = [#Testing rotation + movement accuracy
    Pause(10),
    ImmediateTarget(Vec2(startX, startY + 12), 0.5),
    ImmediateTarget(Vec2(startX + 12, startY + 12), 0.5, True)
]
quick1 = [#Claw drop off person
    Calibrate(clawFront, -50),
    TargetAngle(clawFront, 105, 120),
    Wait(),
    ImmediateTarget(Vec2(52, 17.5), 0.5, True, False, True),
    AlignToAngle(-90, 1),
    Calibrate(clawFront, -100),
    ImmediateTarget(Vec2(5, 5), 5, False, True, True)
]
quick2 = [#Plastic bag
    Calibrate(clawBack, 70),
    ImmediateTarget(Vec2(38, 17), 0.5, True, True),
    ImmediateTarget(Vec2(54, 17), 0.5, True, True),
    ImmediateTarget(Vec2(68, 7), 0.25, True),
    TargetAngle(clawBack, -180, 240),
    ImmediateTarget(Vec2(38, 14.8), 1, False, True, True),
    ImmediateTarget(Vec2(5, 5), 5, False, True, True)
]
quick3 = [#Demo
    Calibrate(clawBack, 70),
    ImmediateTarget(Vec2(15, 5), 1, True, True, True),
    ImmediateTarget(Vec2(23.25, 5), 0.5, True, False, True),
    AlignToAngle(-90, 5),
    AlignToAngle(-150, 5),
    # AlignToAngle(-30, 5),
    # AlignToAngle(-90, 5),
    ImmediateTarget(Vec2(5, 5), 5, True, True, True)
]
quick4 = [#Salvage
    Calibrate(clawBack, 70),
    TargetAngle(clawBack, -150, 120),
    ImmediateTarget(Vec2(30, 12), 0.5, False, True),
    ImmediateTarget(Vec2(45, 12), 0.5, False, True),
    ImmediateTarget(Vec2(50, 5.75), 0.25),
    AlignToAngle(90, 1),
    TargetAngle(clawBack, -120, 120),
    ImmediateTarget(Vec2(44.75, 5.75), 1, True),#Grab
    AlignToAngle(90, 1),
    TargetAngle(clawBack, -210, 120),
    ImmediateTarget(Vec2(45, 5.75), 1, False, True),
    ImmediateTarget(Vec2(45, 12), 1, False, True, True),
    ImmediateTarget(Vec2(30, 12), 1, False, True, True),
    ImmediateTarget(Vec2(5, 5), 3, False, True, True)
]
quick5 = [#Transport
    Calibrate(clawBack, 70),
    Calibrate(clawFront, -50),
    TargetAngle(clawBack, -270, 240),
    ImmediateTarget(Vec2(startX, 37), 0.5, False, True, True),#Begin transport
    AlignToAngle(-90, 1),
    # ImmediateTarget(Vec2(15, 37), 0.5, True, True),
    # AlignToAngle(-90, 1),
    # ImmediateTarget(Vec2(20, 37), 0.5, True, True),
    # AlignToAngle(-90, 1),
    # ImmediateTarget(Vec2(30, 37), 0.5, True, True),
    # AlignToAngle(-90, 1),
    ImmediateTarget(Vec2(50, 37), 0.5, True, True, True),#Very hacky
    # ImmediateTarget(Vec2(59, 38), 0.5, True),
    # ImmediateTarget(Vec2(52.5, 40), 0.5, False),#Back off
    # ImmediateTarget(Vec2(57.5, 42), 0.5, True, True),#End transport
    # ImmediateTarget(Vec2(69, 42), 0.5, True, True)
]
quick6 = [#Compost
    ImmediateTarget(Vec2(38, 11.5), 2, False, True, True),
    ImmediateTarget(Vec2(100, 11.5), 1, False, True, True)
]
stack1 = [#Run #1
    Calibrate(clawBack, 70),
    Calibrate(clawFront, -50),
    TargetAngle(clawBack, -150, 120),
    ImmediateTarget(Vec2(startX, 30.125), 0.25),#The slight difference between above and below is so the approach is straighter.
    AlignToAngle(-90, 1),
    TargetAngle(clawBack, -120, 120),
    ImmediateTarget(Vec2(14.5, 30.5), 0.5, True),#Approach factory
    AlignToAngle(-90, 1),
    TargetAngle(clawBack, -240, 120),
    TargetAngle(clawBack, -150, 120),
    ImmediateTarget(Vec2(startX, 30.25), 1, False, True),
    TargetAngle(clawBack, -270, 120),
    ImmediateTarget(Vec2(12.5, 39.5), 0.5, True, True),#Begin transport
    ImmediateTarget(Vec2(20, 39.5), 0.5, True, True),
    ImmediateTarget(Vec2(30, 40), 0.5, True, True),
    ImmediateTarget(Vec2(57.5, 40), 0.5, True),
    ImmediateTarget(Vec2(52.5, 40), 0.5, False),#Back off
    ImmediateTarget(Vec2(57.5, 42), 0.5, True, True),#End transport
    ImmediateTarget(Vec2(69, 42), 0.5, True, True),
    ImmediateTarget(Vec2(69, 28), 0.5),
    ImmediateTarget(Vec2(59.5, 10), 0.5),
    ImmediateTarget(Vec2(startX, startY), 0.5)
]
stack2 = [

]
stack3 = [#Run #3
    Calibrate(clawBack, 20),
    Calibrate(clawFront, -50),
    ImmediateTarget(Vec2(16, 14), 0.5, False, True),
    ImmediateTarget(Vec2(32, 14), 0.5, False, True),
    ImmediateTarget(Vec2(58, 17.5), 0.5, False, True),
    ImmediateTarget(Vec2(73, 25.5), 0.5), #car task
    ImmediateTarget(Vec2(55.5, 19.5), 0.5, False, True),
    ImmediateTarget(Vec2(59, 12.25), 0.25),
    # TargetAngle(clawFront, 0, 120),
    ImmediateTarget(Vec2(62, 12.25), 0.25),
    AlignToAngle(90, 1),
    # TargetAngle(clawFront, 45, 120),
    ImmediateTarget(Vec2(42,6), 0.5),
]
stacks = [quick1, quick2, quick3, quick4, quick5, quick6]
stackNames = ["Careers", "Plastic Bag", "Demo", "Salvage", "Transport", "Compost"]
#Current command
command = 0

#Fancy
mainFont = Font(family="Terminal", size=15, bold=False, monospace=True)
ev3.screen.set_font(mainFont)

if True:#no im not high, this is just so I can collapse it
    t1 = Thread(target=trackLine)
    t1.daemon = True#haha daemon
    t1.start()

    t2 = Thread(target=updatePosition)
    t2.daemon = True
    t2.start()

    t3 = Thread(target=runDisplay)
    t3.daemon = True
    t3.start()

#Main loop
def main():
    global command
    global cycle
    global lastDeltaTime
    global passedTime
    global i
    global currentTime
    global mainLoopFps
    global suspendDisplay
    #global global global

    suspendDisplay = False

    while True:
        if(Button.DOWN in ev3.buttons.pressed()):
            break

        #Initial stuff
        currentTime = time.time()

        #Command execute
        if(button.pressed()):
            drivetrainLeft.hold()
            drivetrainRight.hold()
        else:
            if(command != 0):
                command.execute()

        #Major update
        if(cycle % 100 == 0):
            i += 1
            mainLoopFps = 100/passedTime
            passedTime = 0
        
        #Command ending
        if(command == 0 and len(stack) > 0):
            command = stack[0]
            stack.pop(0)

        if(command == 0):
            break

        #More timing stuff
        cycle += 1
        lastDeltaTime = time.time() - currentTime
        passedTime += lastDeltaTime

    suspendDisplay = True

def core():
    global stack
    global position
    global lastAngle
    global lastDrivetrainState

    while True:#Inter run loop
        selectedRun = 0
        leftTicks = 0
        rightTicks = 0
        
        updated = True

        while True:#Input loop
            buttons = ev3.buttons.pressed()
            if(Button.LEFT in buttons):
                leftTicks += 1#Little trick I learned back in the JS on Khan Academy days, dead useful.
            else:
                leftTicks = 0
            if(Button.RIGHT in buttons):
                rightTicks += 1
            else:
                rightTicks = 0
            
            if(rightTicks == 1):#This way you only get the initial press
                selectedRun += 1
                updated = True
            if(leftTicks == 1):
                selectedRun -= 1
                updated = True

            if(updated):
                updated = False
                selectedRun %= len(stacks)
                ev3.screen.clear()
                ev3.screen.draw_text(0, 0, "Please select which", Color.BLACK)
                ev3.screen.draw_text(0, 15, "path to execute. ", Color.BLACK)
                ev3.screen.draw_text(0, 30, "Currently selected: " + str(selectedRun+1), Color.BLACK)
                ev3.screen.draw_text(0, 45, stackNames[selectedRun], Color.BLACK)

            if(Button.CENTER in buttons):
                break#No need to do anything special, right to the run we go


        #Once the loop has exited, begin the program
        stack = copy.copy(stacks[selectedRun])

        position = Vec2(startX, startY)#Reset parameters
        lastAngle = 0
        lastDrivetrainState = Vec2(0, 0)
        
        gyro.reset_angle(0)
        drivetrainRight.reset_angle(0)
        drivetrainLeft.reset_angle(0)

        main()

        drivetrainLeft.hold()
        drivetrainRight.hold()

if __name__ == "__main__":
    core()

#End
drivetrainLeft.hold()
drivetrainRight.hold()
#Retain display contents
while True:
    cycle += 1#Do *something*