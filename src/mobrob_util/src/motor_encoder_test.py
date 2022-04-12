#! /usr/bin/python3

from pololu_drv8835_rpi import motors, MAX_SPEED
import time
import numpy as np
import traceback
import yaml
import encoders

## Define which serial port your sensors are on: 
#serial_device_name = '/dev/ttyS0'
#serial_device_name = '/dev/ttyUSB0'


def testMotors(left_dir,right_dir,speed): #Runs the motors for ~2 seconds
    motors.motor1.setSpeed(left_dir*speed)
    motors.motor2.setSpeed(right_dir*speed)
    time.sleep(2.0)
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    time.sleep(.5)

def getEncoderRawValues(): #clears out the serial port and waits for the next encoder values
    [leftEnc,rightEnc] = encoders.readEncoders();
    return [leftEnc,rightEnc]

ENCODER_MOVE_THRESH = 1000;  #counts to move before encoder is considered 'moved'

#states for test.  MOTOR
MOTOR_SPIN_LEFT = 0
MOTOR_SPIN_RIGHT = 1
MOTOR_DIR = 2
ENCODER_DIR = 3

test_over = False;
test_passed = True;

#current state for the test..  Starts at MOTOR_SPIN_LEFT
state = MOTOR_SPIN_LEFT

with open("robot_info.yaml", 'r') as stream:
    robot_info = yaml.safe_load(stream)

print("Starting test in 5 seconds.  Please place your robot on the ground.")
time.sleep(5.0)

robot_info = []

with open("robot_info.yaml", 'r') as stream:
    robot_info = yaml.safe_load(stream)

left_motor_dir = robot_info["left_motor_sign"];
right_motor_dir = robot_info["right_motor_sign"];
left_encoder_dir = robot_info["left_encoder_sign"];
right_encoder_dir = robot_info["right_encoder_sign"];

while not test_over:
    if (state == MOTOR_SPIN_LEFT):
        print("Testing Left Motor Connections")
        testMotors(left_motor_dir,0,95)
        txt = input("did the left motor spin? (y/n)")
        if (txt == "y"):
            print("left motor connection good.")
            state = MOTOR_SPIN_RIGHT;
            continue;
        elif (txt == "n"):
            print("Hardware problem: Check all motor connections and try again (if connections look good, ask a TA for help)")
            test_passed = False;
            break;
    elif (state == MOTOR_SPIN_RIGHT):
        print("Testing Right Motor Connections")
        testMotors(0,right_motor_dir,95)
        txt = input("did the right motor spin? (y/n)")
        if (txt == "y"):
            print("right motor connection good.")
            state = MOTOR_DIR;
            continue;
        elif (txt == "n"):
            print("Hardware problem: Check all motor connections and try again (if connections look good, ask a TA for help)")
            test_passed = False;
            break;
    elif (state == MOTOR_DIR):
        print("Testing Motor Directions...")
        testMotors(left_motor_dir,right_motor_dir,95)
        txt = input("did the robot move forward(ish)? (y/n)")
        if (txt == "y"):
            print("Motor signs good.")
            state = ENCODER_DIR
            continue;
        elif (txt == "n"):
            print("Please flip the signs of left_motor_sign or right_motor_sign in robot_params.yaml for the motors that drove backwards -OR- you can swap the M1 and M2 connections on that motor.  Re-run this script to test again.")
            test_passed = False;
            break;
    elif (state == ENCODER_DIR): #this just tests the encoder direction. pure code.
        print("Testing Encoders...")
        [e0_init,e1_init] = getEncoderRawValues();
        testMotors(left_motor_dir,0,95);  #move left motor only
        [e0_left,e1_left] = getEncoderRawValues();
        testMotors(0,right_motor_dir,95); #move right motor only
        [e0_right,e1_right] = getEncoderRawValues();
        print("encoder cnts: " + str(e0_init) + "," + str(e0_left) + "," + str(e0_right) + " " + str(e1_init) + "," + str(e1_left) + "," + str(e1_right))

        if ((e0_left-e0_init)*left_encoder_dir) > ENCODER_MOVE_THRESH:
            print("left encoder good.")
        elif ((e0_left-e0_init)*left_encoder_dir) < -ENCODER_MOVE_THRESH:
            print("change left_encoder_sign in robot_info.yaml -OR- swap the A and B leads on the left motor. Re-run this script to test again.")
            test_passed = False;
        else: #encoder hasn't moved much.. probably a hardware problem.
            print("Hardware problem: check left encoder wiring and try again (if connections look good, ask a TA for help)")
            test_passed = False;

        if ((e1_right-e1_left)*right_encoder_dir) > ENCODER_MOVE_THRESH:
            print("right encoder good.")
        elif ((e1_right-e1_left)*right_encoder_dir) < -ENCODER_MOVE_THRESH:
            print("change right_encoder_sign in robot_info.yaml -OR- swap the A and B leads on the right motor. Re-run this script to test again.")
            test_passed = False;
        else: #encoder hasn't moved much.. probably a hardware problem.
            print("Hardware problem: check left encoder wiring and try again (if connections look good, ask a TA for help)")
            test_passed = False;    
        test_over = True;


if (test_passed):
    print("Test PASSED! your motors and encoders agree and are going the right direction!")
else:
    print("FAILED.  Please take recommended action and try again!")
