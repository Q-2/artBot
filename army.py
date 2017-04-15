from __future__ import division
from Adafruit_MotorHAT.Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import sys
import logging
import subprocess
import RPi.GPIO as GPIO
import math
import time
import random
import atexit
import Adafruit_PCA9685
import Adafruit_MPR121.MPR121 as Touch
import ast

pwm = Adafruit_PCA9685.PCA9685()

feel  = Touch.MPR121()
    
if not feel.begin():  print("wire connection comm error to capacitive touch mpr121 chip" )

feel.set_thresholds(181, 190)

shspeed = 200
elspeed = 200
calibrationdecay = 0

brushvert = 600
finebrush = 270


servo_min = 120  # 150 Min pulse length out of 4096
servo_max = 600  # 600 Max pulse length out of 4096
uppose = 600
paintpose = 280
brush_stop = 300 # Servo position to determine elbow angle by collision
brush_up = range(paintpose, uppose, 6)
brush_down = range(uppose,paintpose,-6)
brush_levels = brush_down

elbowcalibration = 48
shouldercalibration = 0
elbowcircle = 840 # steps for elbow to complete rev
shouldercircle = 840 # steps for shoulder to complete rev

steptodegree  = 360 / elbowcircle

shoulderStepper = Adafruit_MotorHAT(addr = 0x60)
elbowStepper = Adafruit_MotorHAT(addr = 0x61)

shouldercount = 0
elbowcount = 0
wrapwarn =  0

def turnOffMotors():
    shoulderStepper.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    shoulderStepper.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    shoulderStepper.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    shoulderStepper.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    elbowStepper.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    elbowStepper.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    elbowStepper.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    elbowStepper.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    set_servo_pulse(0, 4096)
    print("Motor Shutdown Complete")


def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def brush(pose):
    pwm.set_pwm(0,0,pose)
    time.sleep(0.75)

# Set frquency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def lower():  # Wrist Down
    for i in brush_down:
       pwm.set_pwm(0, 0, i)
       time.sleep(0.06)

def lift():  # Wrist Up
    for i in brush_up:
        pwm.set_pwm(0, 0, i)
        time.sleep(0.06)
def up():
    for i in brush_up:
       pwm.set_pwm(0, 0, i)
       time.sleep(0.01)
  

def down():
    for i in brush_down:
       pwm.set_pwm(0, 0, i)
       time.sleep(0.01)    

atexit.register(turnOffMotors)

shoulder = shoulderStepper.getStepper(shspeed, 1)  # 200 steps/rev, motor port #1
shoulder.setSpeed(shspeed)             # 30 RPM

elbow = elbowStepper.getStepper(elspeed, 1)  # 200 steps/rev, motor port #1
elbow.setSpeed(elspeed)             # 30 RPM

def speed(val):
    if (val < 2000 and val > 64):
       elbow.setSpeed(val)
       shoulder.setSpeed(val)

def normspeed():
    elbow.setSpeed(elspeed)
    shoulder.setSpeed(shspeed)

def splat():
    pwm.set_pwm(0, 0, 650)
    time.sleep(0.7)
    pwm.set_pwm(0, 0, heavybrush)
    time.sleep(0.7)
    pwm.set_pwm(0,0,550)
    time.sleep(0.6)

def dot():
    pwm.set_pwm(0,0,400)
    time.sleep(.25)
    pwm.set_pwm(0, 0, finebrush)
    time.sleep(.5)

def dab():
    pwm.set_pwm(0,0,300)
    time.sleep(0.5)
    pwm.set_pwm(0, 0, lightbrush) 
    time.sleep(0.5)

def dop():
    pwm.set_pwm(0,0,300)
    time.sleep(0.5)
    pwm.set_pwm(0, 0, mediumbrush)
    time.sleep(0.5)

def drop():
    pwm.set_pwm(0, 0, heavybrush)

def el(move): # elbow left, make fist with left hand.  thumb points rotation
    global elbowcount, calibrationdecay,elbowcalibration
    calibrationdecay = calibrationdecay + move
    touched = False
    pinNow = feel.filtered_data(1)
    #calibrationvalue = 48  used for multipoint calibration which turned out to be a bit mechanically difficult
    calibrationzero = elbowcalibration
    if (pinNow < 184): touched = True

    for i in range(move):
        elbow.step(1, Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE)
        elbowcount += 1
        pinNow = feel.filtered_data(1)   #pin 1 = elbow touch pin
        if (pinNow > 184 and touched):  # just untouched a pin?
            touched = False
        #    elbowcount = int((4*calibrationvalue + 4*calibrationzero+elbowcount)/5) #push towards calibrationon backside of pin
        #    get numbers like 49 from calibration.py program
        #    double point calibration is not easy with flimsy slighlty changing hardware setup I have.  Went to single point zero calibration
        #    and commented out second data point calibration and took out moving average calibration
        if (pinNow < 184 and touched == False): # just hit a pin?
            touched = True                      #reverse truth state & move towards zero
            elbowcount = int(calibrationzero)
            calibrationdecay = 0

def er(move):      # elbow right, grip axis thumb up - thumb points rotation direction.  With autocalibration              
    global elbowcount, calibrationdecay,elbowcalibration
    calibrationdecay = calibrationdecay + move
    touched = False
    pinNow = feel.filtered_data(1)
    #calibrationvalue = 49
    calibrationzero = elbowcalibration
    if (pinNow < 184): touched = True

    for i in range(move):
        elbow.step(1, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)
        elbowcount -= 1
        pinNow = feel.filtered_data(1)
        if (pinNow < 184 and touched == False):  # just hit a pin?
           touched = True
        #    elbowcount = int((calibrationvalue + calibrationzerooffset + elbowcount)/5) #push towards calibration w/ 5xMA
        #get numbers like 48 from physical calibration with calibration.py
        if (pinNow > 184 and touched ): # just untouched pin?
            touched = False
            elbowcount =  int(calibrationzero)
            calibrationdecay=0

def sr(move): # shoulder right, grip axis left hand thumb up & thumb points rotation
    global shouldercount, calibrationdecay,shouldercalibration
    calibrationdecay = calibrationdecay + move
    touched = False
    pinNow = feel.filtered_data(0)  #pin 0 shoulder 1 elbow
    #calibrationvalue = -63
    calibrationzero = shouldercalibration
 
    if (pinNow < 181): touched = True

    for i in range(move):
        shoulder.step(1, Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE)
        shouldercount += 1
        pinNow = feel.filtered_data(0)
        if (pinNow > 184 and touched):  # just untouched a pin?
            touched = False
            shouldercount = int(calibrationzero)
            calibrationdecay = 0                    
        if (pinNow < 184 and touched == False): # just hit a pin?
            touched = True
             #reverse truth state & move towards zero
            #shouldercount =  int((4*calibrationzero+4*calibrationvalue+ shouldercount) / 5)
                                
def sl(move): # shoulder left, grip axis left hand thumb up & thumb points rotation
    global shouldercount, calibrationdecay, shouldercalibration
    calibrationdecay = calibrationdecay + move
    touched = False
    pinNow = feel.filtered_data(0)  #pin 0 shoulder 1 elbow
    #calibrationvalue = -64
    calibrationzero= shouldercalibration
    if (pinNow < 181): touched = True

    for i in range(move):
        shoulder.step(1, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)
        shouldercount -= 1
        pinNow = feel.filtered_data(0)
        if (pinNow > 184 and touched):  # just untouched a pin?
            touched = False
        #    shouldercount = int((4*calibrationzero + shouldercount)/5) #push towards calibration w/ 5xMA
        if (pinNow < 184 and touched == False): # just hit a pin?
            touched = True                      #reverse truth state & move towards zero
            shouldercount =  int(calibrationzero )
            calibrationdecay = 0
        
def untangle():
    global shouldercount, elbowcount, calibrationdecay
    up()
    while (elbowcount > 0 ): er(1)
    while (elbowcount < 0 ): el(1)
    while (shouldercount > 0): sl(1)
    while (shouldercount < 0): sr(1)
    if (calibrationdecay>1000):
         sr(150) #enough swing to hit pins from most zeros and recalibrate
         sl(300)
         sr(150)
         er(150)
         el(300)
         er(150)
         
def blindstart():
    global shoulderA, elbowA, shouldercount, elbowcount
    up()
    sr(200) 
    sl(400)
    er(200)
    el(400)
    untangle()

def dip():
     brush(finebrush)
     er(4)
     el(8)
     er(4)
     lift()

def startFound():
    global elbowsteps
    global shouldersteps
    elbowsteps = 0
    shouldersteps = 0

def move(sho,elb):  #moves elbow first reducing risk of center leg hit
    global shouldercount, elbowcount
    while (elbowcount > elb ): er(1)
    while (elbowcount < elb ): el(1)
    while (shouldercount > sho): sl(1)
    while (shouldercount < sho): sr(1)
    if (shouldercount > 430 or shouldercount < -430 or elbowcount > 430 or elbowcount < -430): untangle()

def smoove(sho, elb):  #smoother move with greater risk of leg hits
    for i in range(int(abs(sho)* abs(elb)) ):  #create shoulder*elbow steps, do step when modulus is zero
        if (i%sho == 0): #not reversed, tricksy modulus algorithmic smoothing smoothing
           if (elbowcount > elb): er(1)
           if (elbowcount < elb): el(1)
        if (i%elb == 0):#not reversed or wrong, elbow modulus for shoulder moves.
           if (shouldercount > sho):
               sl(1)
           if (shouldercount < sho):
               sr(1)
    if (shouldercount > 430 or shouldercount < -430 or elbowcount > 500 or elbowcount < -500): untangle()

def paint(can):
    
    if (can == 1):
       move(-85,0)
       dip()
       
    if (can == 2):
       move(-65,0)
       dip()

    if (can == 3):
       move(-30,0)
       dip()
       
    if (can == 4):
       move(3,0)
       dip()
       
    if (can == 5):
       move(38,0)
       dip()
       
    if (can == 6):
       move(65,0)
       dip()


def xytoSA(x,y):
    riseA = math.atan2(y,x)
    dist1 = math.sqrt(x*x + y*y)
    dist2 = dist1 / 2
    ebbyA = math.acos(dist2)
    shdrstps = 840 * (ebbyA - riseA)/(2*3.141592654)
    return shdrstps

def xytoEA(x,y):
    riseA = math.atan2(y,x)
    dist1 = math.sqrt(x*x + y*y)
    dist2 = dist1 / 2
    ebbyA = math.acos(dist2)
    shdrA = ebbyA - riseA
    elbwstps = 840 * (3.141592654 - 2* ebbyA) / (2*3.141592654)
    return elbwstps

def secs(seconds):
    time.sleep(seconds)



print("Gently move to start position with shoulder and elbow touching sensor pins")

up()
time.sleep(2)
down()
time.sleep(2)
up()
time.sleep(2)
down()
time.sleep(2)
up()
time.sleep(2)
down()
time.sleep(2)
