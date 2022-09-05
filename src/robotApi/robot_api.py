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
time = Timer()
right_attachment = Motor('F')
left_attachment = Motor('E')

#Variable definitions


target = 20
number_of_lines_passed = 0
left_last_line = False
timeout = 0


#Get value function definitions


def get_cm():
    return int(-(get_degree()/(360/17.6)))
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
    reset_all()
def reset_all():
    number_of_lines_passed = 0
    left_last_line = False
    time.reset()
    left.set_degrees_counted(0)
    hub.motion_sensor.reset_yaw_angle()
def reset_line():
    number_of_lines_passed = 0
    left_last_line = False
def reset_degree_left():
    left.set_degrees_counted(0)
def reset_degree_right():
    right.set_degrees_counted(0)
def reset_time():
    time.reset()
def reset_yaw():
    hub.motion_sensor.reset_yaw_angle()
def set_black_line(line):
    global target
    target = line
def on_black_line_left():
    return (color_left.get_reflected_light() <= target)
def on_black_line_right():
    return (color_right.get_reflected_light() <= target)
def on_black_line():
    return (color_left.get_reflected_light()) <= target or (color_right.get_reflected_light()) <= target


#Function definitions start here


#Tested, works, score 90/100, moves smoothly and acurately but can't keep up to sharp turns


def line_follow(stop_condition, power, color, side):
    reset()

    while not stop_condition():
        kp = 0.4
        target_light = 60
        motor_power = int(power)
        if color == 'left':
            correction = kp * (target_light - get_light_left())
        elif color == 'right':
            correction = kp * (target_light - get_light_right())
        else:
            correction = kp * (target_light - get_light_right())
        if side == 'left':
            motor_pair.start_tank_at_power(int(motor_power - correction), int(motor_power + correction))
        else:
            motor_pair.start_tank_at_power(int(motor_power + correction), int(motor_power - correction))
    motor_pair.stop()


#Smoother gradient.


def proportional_move(stop_conditions, power=0, power_function=None):
    global number_of_lines_passed, left_last_line
    reset()
    while not stop_conditions():
        if power_function != None:
            motor_power = power_function()
        else:
            motor_power = power
        correction = 0 - get_yaw()
        motor_pair.start_tank_at_power(int((motor_power + correction)), int((motor_power - correction)))
        if on_black_line_left() and left_last_line:
            number_of_lines_passed += 1
            left_last_line = False
        elif not on_black_line_left():
            left_last_line = True
    motor_pair.stop()


#Tested, works, score 100/100, turns smoothly and acurately


def proportional_turn(degree,minimum_power=25,maximum_power=100):
    reset()
    target_degree = degree
    kp = 0.4
    distance = target_degree - get_yaw()
    minimum_speed = minimum_power
    maximum_speed = maximum_power
    while abs(distance) > 1:
        distance = target_degree - get_yaw()
        output = distance * kp
        if abs(output) > maximum_speed:
            output = maximum_speed * (target_degree / abs(target_degree))
        elif abs(output) < minimum_speed:
            output = minimum_speed * (target_degree / abs(target_degree))
        motor_pair.start_tank(int(output), int(-output))
    motor_pair.stop()


#Tested, doesn't work, 0/100, not smooth and accurate


def line_square(motor_power):
    l_value = 0
    power = int(motor_power)
    motor_pair.start_tank_at_power(power, power)
    wait_until(on_black_line)
    motor_pair.stop()
    if on_black_line_left():
        motor_pair.start_tank_at_power(0,power)
        wait_until(on_black_line_right)
        motor_pair.start_tank_at_power(-power,0)
        wait_until(on_black_line_left)
        wait_until(on_black_line_left, equal_to, False)
        motor_pair.start_tank_at_power(0,-power)
        wait_until(on_black_line_right, equal_to, False)
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_left)
        motor_pair.start_tank_at_power(0,power)
        wait_until(on_black_line_right)
        motor_pair.stop()
    elif on_black_line_right():
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_left)
        motor_pair.start_tank_at_power(0,-power)
        wait_until(on_black_line_right)
        wait_until(on_black_line_right, equal_to, False)
        motor_pair.start_tank_at_power(-power,0)
        wait_until(on_black_line_left, equal_to, False)
        motor_pair.start_tank_at_power(0,power)
        wait_until(on_black_line_right)
        motor_pair.start_tank_at_power(power,0)
        wait_until(on_black_line_left)
        motor_pair.stop()


#Beta-testing



def gradient_move(distance, power):
    proportional_move(lambda : get_cm() >= distance, 0,lambda : power/2 if (get_cm() <= (distance/5) or get_cm() >= ((distance/5)*4)) else power)



#Resets everything


reset()


#List of functions

'''
line_follow(get value1,power, color)
proportional_movement(get value2,power,powerfunction=None)
proportional_turning(degrees)
line_square(power)
gradient_move(distance, power)
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

#Lambda example:
'''
Instead of:
def timmy_mc_boy():
    return changing_variable_twenty_four_seven > another_bozo
is_timmy_bozo(timmy_mc_boy)

Do:
is_timmy_bozo(lambda : changing_variable_twenty_four_seven > another_bozo)
'''

#Start coding here
