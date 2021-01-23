#! /usr/bin/env python

from adafruit_servokit import ServoKit
import time
import json
import os

kit = ServoKit(channels=16) #instantiate pwm driver object; everything is done automatically and will work out of the box for jetson

# channel 0 is throttle
# channel 1 is steering

# right now the steering is actually perfectly calibrated with this library
# however the throttle is still very sensitive, so 0.1 is pretty fast already
# look at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython for more documentation
# specifically the set_pulse_width_range() function; you will use this to build the calibration script

#steering calibration
# Ask user to input values around 0 and 180 to determine where the 
# maximum and minimum steering is. Also ask for values around 90 to determine the "straight" value.

#initialize lists containing the info reqd. to calibrate the steering and throttle (these are values that will likely work for an ackerman steering car)
#Each variable is a list, first value represents the steering/throttle value. Second value is used to indicate if the variable has been calibrated (used later in the script.()
straight = [90.0,'n']
max_right = [170.0,'n']
max_left = [10.0,'n']
abs_max_right = [2000,'n']
abs_max_left = [1000,'n']

neutral_throttle = [0.0,'n'] 
<<<<<<< HEAD
max_throttle = [0.1,'n']
#reverse = [-0.1,'n'] #original implementatio does not use reverse/braking functionality

pause = 1 #number of seconds to pause for the input values to change the motor/servo values
throttle_pause = 3 #Number of seconds to pause for the user to observe throttle output

safe = "n"
while safe != "y" or safe =="n":
     safe = str(input("Is the car off the ground? It will steer and throttle during this script. y/n "))
     if safe == "y":
          pass
     elif safe != "y" or safe != "n":
          safe = input("Please enter y/n: ")
     else:
          print("The calibration script is exiting.")
          exit()
=======
max_throttle = [0.2,'n']
reverse = [-0.5,'n']

pause = .1 #number of seconds to pause for the input values to change the motor/servo values
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde

#start by finding the absolute maximum and minimum steering positions via pwm width values
print('We will now look for the PWM widths that correspond to maximum right and left steering positions possible for the car. Here you''re looking for the angles where the car has maxed out its right and left steering. Later you will decide the steering range for the car.')

if abs_max_right[1] == 'n' or abs_max_left[1] == 'n':
    while abs_max_right[1] == 'n':
        abs_max_right[0] = float(input('Please input a value greater than 1500 and near 2000 to search for the maximum possible right steering angle: '))
        kit.servo[1].set_pulse_width_range(abs_max_left[0],abs_max_right[0])
        kit.servo[1].angle = 180
        decision = str(input('Is this the maximum right steering angle for the vehicle? y/n: '))
        kit.servo[1].angle = 90
        if decision == 'y':
            print('The maximum right steering angle has been saved')
            break

    while abs_max_left[1] == 'n':
        abs_max_left[0] = float(input('Please input a value less then 1500 and near 1000 to search for the maximum possible left steering angle: '))
        kit.servo[1].set_pulse_width_range(abs_max_left[0],abs_max_right[0])
        kit.servo[1].angle = 0
        decision = str(input('Is this the maximum left steering angle for the vehicle? y/n: '))
        kit.servo[1].angle = 90
        if decision == 'y':
            print('The maximum left steering angle has been saved')
            break

# set max right angle
kit.servo[1].angle = 90 #set steering to the approximate straight value
while max_right[1] == 'n':
<<<<<<< HEAD
    max_right[0] = float(input('Input a value between 90 and 180 to search for the cars maximum right steering position: '))
    if max_right[0] < 0 or max_right[0] > 180:
        print('Please enter a number between 0 and 180.')
    else:
        kit.servo[1].angle = max_right[0] #publish the steering value to the car to observe the steering angle
        time.sleep(pause) #pause for "pause" seconds to allow the servo to move
        max_right[1] = str(input('Do you want to store this value as the maximum right steering angle? y/n: '))
=======
    max_right[0] = input('Input a value between 90 and 180 to search for the cars maximum right steering position: ')
    kit.servo[1].angle = max_right[0] #publish the steering value to the car to observe the steering angle
    time.sleep(pause) #pause for "pause" seconds to allow the servo to move
    max_right[1] = input('Do you want to store this value as the maximum right steering angle? y/n')
    if max_right[1] == 'y':
        print('The maximum right steering angle has been set to: ',max_right[0])
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde
        kit.servo[1].angle = 90 #set steering back to straight
    if max_right[1] == 'y':
        print('The maximum right steering angle has been set to: ',max_right[0])
        break
<<<<<<< HEAD
#    elif max_right[1] != 'y':
#        while max_right[1] != 'y' or max_right[1] != 'n':
#            print('Please enter y to set the max value, or n to try a new value.')
#            max_right[1] = input('Do you want to store the current steering angle #as the maximum right steering angle? y/n ')
#        kit.servo[1].angle = 90

# set max left angle
while max_left[1] == 'n':
    max_left[0] = float(input('Input a value between 0 and 90 to search for the cars maximum left steering position: '))
    if max_left[0] < 0 or max_left[0] > 180:
        print('Please enter a number between 0 and 180.')
    else:
        kit.servo[1].angle = max_left[0] #publish the steering value to the car to observe the steering angle
        time.sleep(pause) #pause for "pause" seconds to allow the servo to move
        max_left[1] = str(input('Do you want to store this value as the maximum left steering angle? y/n: '))
=======
    elif max_right[1] != 'y' or max_right[1] != 'n':
        kit.servo[1].angle = 90 #set steering back to straight
        while max_right[1] != 'y' or max_right[1] != 'n':
            print('Please enter y to set the max value, or n to try a new value.')
            max_right[1] = input('Do you want to store the current steering angle as the maximum right steering angle? y/n ')

# set max left angle
while max_left[1] == 'n':
    max_left[0] = input('Input a value between 0 and 90 to search for the cars maximum left steering position: ')
    kit.servo[1].angle = max_left[0] #publish the steering value to the car to observe the steering angle
    time.sleep(pause) #pause for "pause" seconds to allow the servo to move
    max_left[1] = input('Do you want to store this value as the maximum left steering angle? y/n')
    if max_left[1] == 'y':
        print('The maximum left steering angle has been set to: ',max_left[0])
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde
        kit.servo[1].angle = 90 #set steering back to straight
    if max_left[1] == 'y':
        print('The maximum left steering angle has been set to: ',max_left[0])
        break
<<<<<<< HEAD
#    elif max_left[1] != 'y' or max_left[1] != 'n':
#        while max_left[1] != 'y' or max_left[1] != 'n':
#            print('Please enter y to set the max value, or n to try a new value.')
#            max_left[1] = input('Do you want to store the current steering angle as the maximum left steering angle? y/n ')
 
# set straight forward
while straight[1] == 'n':
    straight[0] = float(input('Input a value between 0 and 180 to search for the cars straight steering position: '))
    if straight[0] < 0 or straight[0] > 180:
        print('Please enter a number between 0 and 180.')
    else:
        kit.servo[1].angle = straight[0] #publish the steering value to the car to observe the steering angle
        time.sleep(pause) #pause for "pause" seconds to allow the servo to move
        straight[1] = str(input('Do you want to store this value as the straight steering angle? y/n: '))
=======
    elif max_left[1] != 'y' or max_left[1] != 'n':
        kit.servo[1].angle = 90 #set steering back to straight
        while max_left[1] != 'y' or max_left[1] != 'n':
            print('Please enter y to set the max value, or n to try a new value.')
            max_left[1] = input('Do you want to store the current steering angle as the maximum left steering angle? y/n ')
 
# set straight forward
while straight[1] == 'n':
    straight[0] = input('Input a value between 0 and 180 to search for the cars straight steering position: ')
    kit.servo[1].angle = straight[0] #publish the steering value to the car to observe the steering angle
    time.sleep(pause) #pause for "pause" seconds to allow the servo to move
    straight[1] = input('Do you want to store this value as the straight steering angle? y/n')
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde
    if straight[1] == 'y':
        print('The straight steering angle has been set to: ',straight[0])
        kit.servo[1].angle = float(straight[0]) #set steering back to straight
        break
<<<<<<< HEAD
#    elif straight[1] != 'y' or straight[1] != 'n':
#        while straight[1] != 'y' or straight[1] != 'n':
#            print('Please enter y to set the max value, or n to try a new value.')
#            straight[1] = input('Do you want to store the current steering angle as the straight steering angle? y/n ')
=======
    elif straight[1] != 'y' or straight[1] != 'n':
        kit.servo[1].angle = 90 #set steering back to straight
        while straight[1] != 'y' or straight[1] != 'n':
            print('Please enter y to set the max value, or n to try a new value.')
            straight[1] = input('Do you want to store the current steering angle as the straight steering angle? y/n ')
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde



# kit.continuous_servo[0].throttle = 0.1 #throttle is forward (pretty fast)
# kit.continuous_servo[0].throttle = -0.5 #throttle electronically brakes (it will e-brake when pwm goes from high number like 0.2 suddenly to low number like -0.5)
# kit.continuous_servo[0].throttle = 0 #throttle is at rest (but not braking so similar to neutral)

# set neutral throttle
while neutral_throttle[1] == 'n':
<<<<<<< HEAD
    neutral_throttle[0] = float(input('Input a value near zero to search for the point at which the throttle is at zero output: '))
    if neutral_throttle[0] < 0 or neutral_throttle[0] > 1:
        print('Please enter a number between 0 and 1.')
    else:
        kit.continuous_servo[0].throttle = float(neutral_throttle[0]) #publish the throttle value to the car for observation
        time.sleep(throttle_pause) #pause for "pause" seconds to allow the throttle to spin up
        kit.continuous_servo[0].throttle = 0 #set throttle back to zero
    neutral_throttle[1] = str(input('Do you want to store this value as the neutral throttle? y/n: '))
=======
    neutral_throttle[0] = input('Input a value near zero to search for the point at which the throttle is just at zero output: ')
    kit.continuous_servo[0].throttle = neutral_throttle[0] #publish the throttle value to the car for observation
    time.sleep(pause) #pause for "pause" seconds to allow the throttle to spin up
    neutral_throttle[1] = input('Do you want to store this value as the neutral throttle? y/n')
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde
    if neutral_throttle[1] == 'y':
        print('The neutral throttle value has been set to: ',neutral_throttle[0])
        kit.continuous_servo[0].throttle = 0 #set throttle back to neutral.
        break
<<<<<<< HEAD
#    elif neutral_throttle[1] != 'y' or neutral_throttle[1] != 'n':
#        kit.continuous_servo[0].throttle = float(neutral_throttle[0]) #set throttle back to neutral.
#        while neutral_throttle[1] != 'y' or neutral_throttle[1] != 'n':
#            print('Please enter y to set the neutral throttle value, or n to try a new value.')
#            neutral_throttle[1] = input('Do you want to store the entered value as the neutral throttle? y/n ')
        
# set max throttle
while max_throttle[1] == 'n':
    max_throttle[0] = float(input('Input a value between 0 and 1 to set the max throttle: '))
    if max_throttle[0] < 0 or max_throttle[0] > 1:
        print('Please enter a number between 0 and 1.')
    else:
        kit.continuous_servo[0].throttle = float(max_throttle[0]) #publish the throttle value to the car for observation
        time.sleep(throttle_pause) #pause for "pause" seconds to allow the throttle to spin up
        kit.continuous_servo[0].throttle = 0 #set throttle back to zero
    max_throttle[1] = str(input('Do you want to store this value as the max throttle? y/n'))
=======
    elif neutral_throttle[1] != 'y' or neutral_throttle[1] != 'n':
        kit.continuous_servo[0].throttle = neutral_throttle[0] #set throttle back to neutral.
        while neutral_throttle[1] != 'y' or neutral_throttle[1] != 'n':
            print('Please enter y to set the neutral throttle value, or n to try a new value.')
            neutral_throttle[1] = input('Do you want to store the entered value as the neutral throttle? y/n ')
        
# set max throttle
while max_throttle[1] == 'n':
    max_throttle[0] = input('Input a value between 0 and 1 to set the max throttle: ')
    kit.continuous_servo[0].throttle = max_throttle[0] #publish the throttle value to the car for observation
    time.sleep(pause) #pause for "pause" seconds to allow the throttle to spin up
    max_throttle[1] = input('Do you want to store this value as the max throttle? y/n')
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde
    if max_throttle[1] == 'y':
        print('The max throttle value has been set to: ',max_throttle[0])
        kit.continuous_servo[0].throttle = 0 #set throttle back to neutral.
        break
<<<<<<< HEAD
#    elif max_throttle[1] != 'y' or max_throttle[1] != 'n':
#        kit.continuous_servo[0].throttle = float(neutral_throttle[0]) #set throttle back to neutral.
#        while max_throttle[1] != 'y' or max_throttle[1] != 'n':
#            print('Please enter y to set the neutral throttle value, or n to try a new value.')
#            max_throttle[1] = input('Do you want to store the entered value as the max throttle? y/n ')

# set reverse throttle
#while reverse[1] == 'n':
#    reverse[0] = input('Input a value between 0 and -1 to set the max reverse throttle: ')
#    kit.continuous_servo[0].throttle = float(reverse[0]) #publish the throttle value to the car for observation
#    time.sleep(pause) #pause for "pause" seconds to allow the throttle to spin up
#    reverse[1] = input('Do you want to store this value as the max reverse throttle? y/n')
#    if reverse[1] == 'y':
#        print('The max reverse throttle value has been set to: ',reverse[0])
#        kit.continuous_servo[0].throttle = float(neutral_throttle[0]) #set throttle back to neutral.
#        break
#    elif reverse[1] != 'y' or reverse[1] != 'n':
#        kit.continuous_servo[0].throttle = float(neutral_throttle[0]) #set throttle back to neutral.
#        while reverse[1] != 'y' or reverse[1] != 'n':
#            print('Please enter y to set the max reverse throttle value, or n to try a new value.')
#            reverse[1] = input('Do you want to store the entered value as the max reverse throttle? y/n ')
=======
    elif max_throttle[1] != 'y' or max_throttle[1] != 'n':
        kit.continuous_servo[0].throttle = neutral_throttle[0] #set throttle back to neutral.
        while max_throttle[1] != 'y' or max_throttle[1] != 'n':
            print('Please enter y to set the neutral throttle value, or n to try a new value.')
            max_throttle[1] = input('Do you want to store the entered value as the max throttle? y/n ')

# set reverse throttle
while reverse[1] == 'n':
    reverse[0] = input('Input a value between 0 and -1 to set the max reverse throttle: ')
    kit.continuous_servo[0].throttle = reverse[0] #publish the throttle value to the car for observation
    time.sleep(pause) #pause for "pause" seconds to allow the throttle to spin up
    reverse[1] = input('Do you want to store this value as the max reverse throttle? y/n')
    if reverse[1] == 'y':
        print('The max reverse throttle value has been set to: ',reverse[0])
        kit.continuous_servo[0].throttle = neutral_throttle[0] #set throttle back to neutral.
        break
    elif reverse[1] != 'y' or reverse[1] != 'n':
        kit.continuous_servo[0].throttle = neutral_throttle[0] #set throttle back to neutral.
        while reverse[1] != 'y' or reverse[1] != 'n':
            print('Please enter y to set the max reverse throttle value, or n to try a new value.')
            reverse[1] = input('Do you want to store the entered value as the max reverse throttle? y/n ')
>>>>>>> 69ec4fe6428f6f36a838a6e3f14f35f44caf4cde

decision = 'n'
parent_path = os.path.dirname(os.getcwd()) #this assumes the script will be run from the .../scripts directory with the json file located in a directory, one directory above
json_path = os.path.join(parent_path,'json_files','car_config.json')
while decision == 'n'or decision != 'y': #write the calibration values to the text caliration file
    decision = input('Do you want to write these values to the car configuration text file? y/n?')
    if decision == 'y': 
        config_file = open(json_path,"w")
        calibration_pack = {'straight':straight[0],'max_left':max_left[0],'max_right':max_right[0],'max_throttle':max_throttle[0],'neutral_throttle':neutral_throttle[0]}
        json.dump(calibration_pack,config_file)
        config_file.close()
        print('The config file has been updated.')
    elif decision == 'n':
        print('The calibration values were not saved to car_config.json.')
        break
    else:
        print('Please select either y or n.')
