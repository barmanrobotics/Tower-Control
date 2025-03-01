#Python script to control a stepper motor from limit switch

#importing MAVLinks libary
pip install pymavlink
from pymavlink import mavutil
#setting up connection to the vehicle
mavConnection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
#waiting for heartbeat
mavConnection.wait_heartbeat()
print("MAVLink connected")

#importing GPIO library
pip install gpiozero

from gpiozero import Button
from time import sleep
from signal import pause
import RPi.GPIO as GPIO
from RPIMotorLib import RPIMotorLib

#setting GPIO pins
limitSwitchPin = 23

#setting motor config
directionPin = 20
stepPin = 21
enablePin = 12 #set to None if not used
stepsPerRev = 200
leadscrewPitch = 8
targetDistance = 18 #in mm
stepsPerMM = stepsPerRev / leadscrewPitch
targetSteps = int(stepsPerMM * targetDistance)
delay = 0.002

#initializing stepper motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(directionPin, GPIO.OUT)
GPIO.setup(stepPin, GPIO.OUT)
GPIO.setup(enablePin, GPIO.OUT)
GPIO.output(enablePin, GPIO.LOW)

#initializing limit switch
limitSwitch = Button(limitSwitchPin, pull_up=False)

#function to move stepper until limit switch is pressed
def moveStepper(steps, direction):
    GPIO.output(directionPin, direction)
    print("Moving stepper")
    for x in range(steps):
        if limitSwitch.is_pressed:
            print("Limit switch pressed, stopping stepper")
            break
        GPIO.output(stepPin, GPIO.HIGH)
        sleep(delay)
        GPIO.output(stepPin, GPIO.LOW)
        sleep(delay) 
    print("Stepper motor completed")

def extendStepperUntilLimit():
    moveStepper(targetSteps, GPIO.HIGH)

def retractStepperUntilLimit():
    moveStepper(targetSteps, GPIO.LOW)

#function to listen for MavLink messages
def listenForMavLinkMessages():
    print("Listening for MavLink messages")
    while True:
        message = mavConnection.recv_match(blocking=True)
        if message.get_type() == "COMMAND_LONG" and message.command == 300:
            if message.param1 == 1: #for ex ArUco dectected at the correct altitude
                extendStepperUntilLimit()
            elif message.param1 == 2: #for ex drone took off... 
                retractStepperUntilLimit()

#running the script
print("Waiting for MavLink messages")
listenForMavLinkMessages()

