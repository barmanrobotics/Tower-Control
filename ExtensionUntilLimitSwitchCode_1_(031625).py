# import the necessary packages
import RPi.GPIO as GPIO
import time
import serial

# make sure to have GPIO on the raspi: sudo apt-get install python3-rpi.gpio python3-serial

# setting up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# defining GPIO pins
# for the motor driver: assuming use of step_pin + dir_pin (no microstepping)
STEP_PIN = 17    # GPIO pin connected to TMC2209 STEP
DIR_PIN = 27     # GPIO pin connected to TMC2209 DIR
LIMIT_SWITCH_PIN = 22  # GPIO pin connected to limit switch signal

# setting up GPIO pins
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # assuming the switch is normally open so made a pull-up 

# set the direction (to move in the forward direction first)
GPIO.output(DIR_PIN, GPIO.HIGH)  

# UART setup for TMC2209
ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # connecting to UART on Raspberry Pi

# controlling the stepper motor
# I have it set to constantly move the motor until the limit switch is hit once called
def step_motor():
    while True:
        # if the limit switch is pressed, stop the motor
        # I AM ASSUMING LOW SIGNAL WHEN PRESSED
        if GPIO.input(LIMIT_SWITCH_PIN) == GPIO.LOW:
            print("Limit switch hit, stopping motor.")
            break
        
        # pulsing the step pin to move the motor
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.001)  # change this value to adjust speed
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.001)  # change this value to adjust speed

# main execution if wanting to run this code directly
if __name__ == "__main__":
    print("Motor will extend until the limit switch is triggered.")
    try:
        step_motor()
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        # clean up and close the serial connection
        GPIO.cleanup()
        ser.close()
        print("GPIO and serial connection cleaned up.")
