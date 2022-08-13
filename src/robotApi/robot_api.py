from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from spike.operator import equal_to, not_equal_to


#Change log
'''
v1.0 release
v1.1 fixed minor issues
v1.2 fixed proportional turning to be more accurate
v1.3 fixed minor issues in line follow
'''


#Port definitions


hub = PrimeHub()
left = Motor('A')
right = Motor('B')
motor_pair = MotorPair('A','B')
color_left = ColorSensor('C')
color_right = ColorSensor('D')
time = Timer()


#Variable definitions


number_of_lines_passed = 0
left_last_line = False


#Get value function definitions


def get_light_left():
    return color_left.get_reflected_light()
def get_light_right():
    return color_right.get_reflected_light()
def get_lines():
    return number_of_lines_passed
def get_rotation():
    return int(left.get_degrees_counted() / 360)
def get_degree():
    return left.get_degrees_counted()
def get_yaw():
    return hub.motion_sensor.get_yaw_angle()
def get_time():
    return time.now()
def reset():
    number_of_lines_passed = 0
    left_last_line = False
    time.reset()
    left.set_degrees_counted(0)
    hub.motion_sensor.reset_yaw_angle()
def on_black_line_left():
    return (color_left.get_reflected_light() < 40)
def on_black_line_right():
    return (color_right.get_reflected_light() < 40)
def on_black_line():
    return (color_left.get_reflected_light()) < 40 or (color_right.get_reflected_light()) < 40

#Ignore line ends here


#Defining functions start here


#Tested, works, score 90/100, moves smoothly and acurately but can't keep up to sharp turns


def line_follow(stop_condition, number, power, color):
    reset()

    while stop_condition() < number:
        kp = 0.55
        target_light = 66
        motor_power = power
        if color == 'left':
            correction = kp * (target_light - get_light_left())
        elif color == 'right':
            correction = kp * (target_light - get_light_right())
        else:
            correction = kp * (target_light - get_light_right())
        motor_pair.start_tank_at_power(int(motor_power + correction), int(motor_power - correction))
    motor_pair.stop()


#Tested, works, score 95/100, moves smoothly and acurately


def proportional_movement(stop_conditions, number, power):
    global number_of_lines_passed, left_last_line
    reset()

    while True:
        motor_power = power
        correction = 0 - get_yaw()
        motor_pair.start_tank_at_power(int(motor_power + correction), int(motor_power - correction))
        if on_black_line_left() and left_last_line:
            number_of_lines_passed += 1
            left_last_line = False
        elif not on_black_line_left():
            left_last_line = True
        if stop_conditions() >= number:
            break
        else:
            pass
    motor_pair.stop()


#Tested, works, score 100/100, turns smoothly and acurately


def proportional_turning(degree):
    reset()
    target_degree = degree
    kp = 0.6
    distance = target_degree - get_yaw()
    minimum_speed = 30
    maximum_speed = 100
    while abs(distance) > 0:
        distance = target_degree - get_yaw()
        output = distance * kp
        if abs(output) >= maximum_speed:
            output = maximum_speed * (target_degree / abs(target_degree))
        elif (abs(output) < maximum_speed) and (abs(output) > minimum_speed):
            output = abs(distance) * (target_degree/ abs(target_degree))
        elif abs(output) <= minimum_speed:
            output = minimum_speed * (target_degree / abs(target_degree))
        motor_pair.start_tank_at_power(int(output), int(-output))
    motor_pair.stop()


#Tested, works, 90/100, smooth and acurate but needs dark line for maximum effect


def line_square(power):
    motor_pair.start_tank_at_power(power, power)
    wait_until(on_black_line)
    motor_pair.stop()
    if on_black_line_left():
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_right)
        motor_pair.start_tank_at_power(-power,0)
        wait_until(on_black_line_left, equal_to, False)
        motor_pair.start_tank_at_power(0,-power)
        wait_until(on_black_line_right, equal_to, False)
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_left)
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_right)
    elif on_black_line_right():
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_left)
        motor_pair.start_tank_at_power(0,-power)
        wait_until(on_black_line_right, equal_to, False)
        motor_pair.start_tank_at_power(-power,0)
        wait_until(on_black_line_left, equal_to, False)
        motor_pair.start_tank_at_power(0,power)
        wait_until(on_black_line_right)
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_left)
    motor_pair.stop()


#Reset everything


reset()


#List of functions

'''
line_follow(get value1, number, power, color)
proportional_movement(get value2, number, power)
proportional_turning(degrees)
line_square(power)
reset()
'''

#List of get value functions

'''
get_yaw()
get_time()
get_lines()
get_rotation()
get_degree()
get_light_left()
get_light_right()
on_black_line_left()
on_black_line_right()
on_black_line()
'''

#Start coding here

