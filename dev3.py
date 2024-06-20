#!/usr/bin/env micropython

import threading
from time import sleep
from sys import stderr

from ev3dev2.motor import ( # type: ignore
    OUTPUT_A,
    OUTPUT_B,
    OUTPUT_C,
    OUTPUT_D,
    MediumMotor,
    MoveTank,
    SpeedPercent,
)
from ev3dev2.sensor import Sensor, INPUT_1 # type: ignore
from ev3dev2.button import Button # type: ignore
from ev3dev2.port import LegoPort # type: ignore

from ev3dev2.sound import Sound # type: ignore

spkr = Sound()

#CONTASTANTS
CMRPOWER = 4
TURNPOWER = 15

# GLOBAL VARIABLES
global g_ballDirection, g_compassAngle, g_compassOffset , g_goalSignatures, g_numberOfBlobs, g_slowDribbler, g_ballControl, HasCalibrated
g_ballDirection = 0
g_compassAngle = 0
g_compassOffset = 0
g_goalSignatures = 0
g_numberOfBlobs = 0
g_slowDribbler = False
g_ballControl = False
HasCalibrated = False

#Inizializzations:
#-Setting Pixy
in1 = LegoPort(INPUT_1)
in1.mode = "auto"
sleep(2.0)
pixy = Sensor(INPUT_1)
pixy.mode = "SIG1"

#-Setting Seeker
seeker = Sensor(driver_name="ht-nxt-ir-seek-v2")
seeker.mode = "AC-ALL"

#-Setting Compass
compass = Sensor(driver_name="ht-nxt-compass")
compass.mode = "COMPASS"

#-Setting Motors
tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
dribbler1 = MediumMotor(OUTPUT_B)
dribbler2 = MediumMotor(OUTPUT_C)


BALL_MOTORSPOWER_MATRIX = [
    [100, -100],  # Ball Position = 0 Behind or Didn't Find
    [-70, 70],  # Ball Position = 1 Almost Behind
    [-25, 25],  # Ball Position = 2 
    [-15, 25],  # Ball Position = 3 Left
    [0, 8],  # Ball Position = 4
    [100, 100],  # Ball Position = 5 In Front
    [8, 0],  # Ball Position = 6
    [25, -15],  # Ball Position = 7 Right
    [25, -25],  # Ball Position = 8
    [70, -70],  # Ball Position = 9 Almost Behind
]

#MOTOR SPEEDS FUNCTIONS (INPUT IN PERCENTAGE)
#Function:
def Motors_Speed(left_motor_speed: int, right_motor_speed: int):
    tank_drive.on(SpeedPercent((left_motor_speed)), SpeedPercent((right_motor_speed)))


#function:
def Dribbler_Speed(motor_speed: int):
    dribbler1.on(SpeedPercent(motor_speed))
    dribbler2.on(SpeedPercent(motor_speed))


#SENSORS VALUE TASKS:

#Task:
#Task for updating Compass values
#The compass with these drivers uses absolute nord, takes the g_compassOffset as the current angle and create a relative Nord on the g_compassAngle
def Compass():
    global g_compassAngle
    while True:
        try:
            temp = compass.value(0) - g_compassOffset
            if temp < -180:
                g_compassAngle = temp + 360
                
            elif temp > 180:
                g_compassAngle = temp - 360
            else:
                g_compassAngle = temp
        except:
            continue

#Task:
#Task for updating Seeker values
def IrSeeker():
    global g_ballDirection
    while True:
        try:
            g_ballDirection = seeker.value(0) #Can be 0 to 9, Ball positions are in the matrix 
        except:
            continue
        
#Task:
#Task for updating Camera values
def Camera():
    global g_goalSignatures
    global g_numberOfBlobs
    while True:
        try:
            g_numberOfBlobs = pixy.value(0) #IDFK, if it's more than 127 the goal's center is on the right, otherwise is on the left and the goal is between 150 and 80 (approx.)
            g_goalSignatures = pixy.value(1) #Quantity of Signatures, if it's 0 the robot doesnt see the goal, if its 1 it does see the goal, 
                                             #if its more than 1... its a fucking problem --> ricalibrate the camera
        except:
            continue


#GAMEPLAY TASKS:

#Task
#Check if the robot has the control of the ball with the dribbler
def Check_Ball_Control():
    global g_slowDribbler
    global g_ballControl
    while True:
        try:
            if (dribbler1.speed < 1150):
                g_slowDribbler = True
                sleep(0.5)
                if g_slowDribbler and (g_ballDirection == 5 or g_ballDirection == 6 or g_ballDirection == 4):
                    g_ballControl = True
                else:
                    g_ballControl = False
            else:
                g_slowDribbler = False
        except:
            continue
  
#Task      
def Gameplay():
    while True:
        if not g_ballControl:
            Motors_Speed(
                        BALL_MOTORSPOWER_MATRIX[g_ballDirection][0],
                        BALL_MOTORSPOWER_MATRIX[g_ballDirection][1],
                    )
        else:
            # if g_goalSignatures == 1: #if the robot sees the goal
            #     if g_numberOfBlobs > 150: #if the goal is on the left
            #             Motors_Speed(-CMRPOWER, CMRPOWER)
            #             sleep(0.1)
            #     elif g_numberOfBlobs < 80: #if the goal is on the right
            #             Motors_Speed(CMRPOWER, -CMRPOWER)
            #             sleep(0.1)
            #     else: #if the goal is in front of the robot
            #         Motors_Speed(100, 100)
            #         sleep(0.5)
            # else: #if the robot doesn't see the goal
                if g_compassAngle > 30 and g_goalSignatures == 0:
                    Motors_Speed(TURNPOWER, -TURNPOWER)
                    sleep(0.2)
                elif g_compassAngle < -30 and g_goalSignatures == 0:
                    Motors_Speed(-TURNPOWER, TURNPOWER)
                    sleep(0.2)
                else:
                    continue
    
        

def initialization():
    global g_compassOffset
    print("PRONTO", file=stderr)
    spkr.beep()
    hasCalibrated = False
    while True:
        if btn.down:
            g_compassOffset = compass.value(0)
            hasCalibrated = True
        if (btn.right or btn.left) and hasCalibrated:
            if btn.right:
                pixy.mode = "SIG1"
            if btn.left:
                pixy.mode = "SIG2"
            break
        sleep(0.01)
    Dribbler_Speed(100)

task_seeker = threading.Thread(target=IrSeeker)
task_compass = threading.Thread(target=Compass)
task_camera = threading.Thread(target=Camera)
task_ballcontrol = threading.Thread(target=Check_Ball_Control)
task_gameplay = threading.Thread(target=Gameplay)

if __name__ == "__main__":
    btn = Button()
    initialization()
    task_seeker.start()
    task_compass.start()
    task_camera.start()
    task_ballcontrol.start()
    task_gameplay.start()
    while True:
        sleep(0.01)