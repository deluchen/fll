from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from spike.operator import equal_to, not_equal_to

#Port definitions

hub = PrimeHub()
left = Motor('A')
right = Motor('B')
motor_pair = MotorPair('A','B')
color_left = ColorSensor('C')
color_right = ColorSensor('D')
timer = Timer()
#Variable definitions

timeout_seconds = 0

#Retrive value function definitions

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
    return timer.now()
def reset():
    number_of_lines_passed = 0
    left_last_line = False
    timer.reset()
    left.set_degrees_counted(0)
    hub.motion_sensor.reset_yaw_angle()
def on_black_line_left():
    return (color_left.get_reflected_light() < 30)
def on_black_line_right():
    return (color_right.get_reflected_light() < 30)
def on_black_line():
    return (color_left.get_reflected_light()) < 40 or (color_right.get_reflected_light()) < 40

#Defining other functions start here

#Proportional line following:

def line_follow(stop_condition, power=35, color_sensor="left"):
    reset()

    while not stop_condition():
        kp = 0.55
        target_light = 66
        motor_power = power
        if color_sensor == 'left':
            correction = kp * (target_light - get_light_left())
        elif color_sensor == 'right':
            correction = kp * (target_light - get_light_right())
        motor_pair.start_tank_at_power(int(motor_power + correction), int(motor_power - correction))
    motor_pair.stop()

#Stop on line

def stop_on_line(stop_conditions, motor_power=50):
    number_of_lines_passed = 0
    left_last_line = False
    while True:
        correction = 0 - get_yaw()
        motor_pair.start_tank_at_power(int(motor_power + correction), int(motor_power - correction))
        if on_black_line_left() and left_last_line:
            number_of_lines_passed += 1
            left_last_line = False
        elif not on_black_line_left():
            left_last_line = True
        if stop_conditions():
            break
        else:
            pass
    motor_pair.stop()

#Proportional turning:

def proportional_turning(target_degree=0):
    reset()
    kp = 0.6
    distance = target_degree - get_yaw()
    minimum_speed = 45
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

#Line squaring

def line_square(power=50):
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

#Proportional straight movement

def proportional_straight_movement(stop_conditions,power=50):
    while True:
        motor_power = power
        correction = 0 - get_yaw()
        motor_pair.start_tank_at_power(int(motor_power + correction), int(motor_power - correction))
        if stop_conditions():
            break

def timeout_on():
    if timer.now() >= timeout_seconds:
        return True
    else:
        return False

#Reset everything

reset()

'''
FUNCTIONS:
line_follow(stop_condition, power=50, color_sensor="right")
stop_on_line(stop_conditions, motor_power=50)
proportional_turning(degrees=0)
line_square(power=50)
proportional_straight_movement(stop_conditions,power=50)
timeout_on(time_seconds)
reset()
'''

'''
RETRIVE VALUE FUNCTIONS:
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

'''
timer.reset()
timeout_seconds = 3
proportional_straight_movement(timeout_on,69)
stop_on_line(on_black_line,35)
line_square()
timer.reset()
timeout_seconds = 0.35
proportional_straight_movement(timeout_on,35)
proportional_turning(-35)
hub.motion_sensor.reset_yaw_angle()
timer.reset()
timeout_seconds = 1
proportional_straight_movement(timeout_on,50)
motor_pair.stop()
'''
