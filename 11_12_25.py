from hub import port, motion_sensor, sound
import runloop, motor_pair, motor, color_sensor
import math, color, time

#port assignment

l_wheel = port.A
r_wheel = port.C
ground_reader = port.F

drive_motors = motor_pair.PAIR_1

side_arm = port.E
front_reader = port.B
front_arm = port.D

C_WHITE, C_BLACK = 100, 10

wheel_radius = 4 #in cm
track = 13
track_radius = track/2

list_ry=(9,7)
list_gw=(0,6,10)
log_gw=[]
#Tusk arm angle (port.B):
'''
    Ball grab angle: 60
    Red/Yellow grab angle: 60 -> 0 -> 160 (-200) (total: -260)
    Ball Release Angle: 160)-> 0 (-160) -> 60 (-460)
'''

#Side arm angle (port.F):
'''
    Panel angle: 30
    G/W Grab angle: 60
'''


def initialize():
    motor_pair.unpair(motor_pair.PAIR_1)
    motor_pair.pair(motor_pair.PAIR_1, l_wheel, r_wheel)
    # motion_sensor.set_yaw_face(motion_sensor.FRONT)





def motor_position():
    l = motor.relative_position(l_wheel)
    r = motor.relative_position(r_wheel)
    return l + r




async def front_arm_movement(angle, vel = 750):
    await motor.run_for_degrees(front_arm, angle, vel, stop = motor.HOLD)

async def side_arm_movement(angle, velocity = 750):
    '''
    angle: INT
    velocity: Default of 750
    '''
    await motor.run_for_degrees(side_arm, angle, velocity, stop=motor.HOLD)

def fling(angle, velocity):
    start = motor.relative_position(side_arm)
    current = 0
    print("Fling initiated.")
    print("Starting at:", current)
    while(abs(current - start) < angle):
        motor.set_duty_cycle(side_arm, velocity * 100)
        current = motor.relative_position(side_arm)
        print(current)
    motor.stop(side_arm, stop = motor.HOLD)

async def reset_bearing():
    await motor_pair.move_tank_for_degrees(drive_motors, int(19.2*30), -800, 0)
    await motor_pair.move_tank_for_degrees(drive_motors, int(19.2*30), 0, -800)

async def blind_reset():
    await motor_pair.move_tank_for_degrees(drive_motors, 10, 200, 200)






















# =============================================================================
#                                SETUP
# =============================================================================

#Port Assignment

# Port B and F are for front effectors

wheel_radius = 2.8 # Lego Small Wheel has a diameter of 5.6 cm -> Radius = 2.8 cm (3.5 modules)
track = 19.2 # Distance between the contacts of the wheels. 24 modules long


def setup():
    """
    Initializes system-wide constants and resets.
    Calibrates light sensors, gyro, and timing.
    """
    global lightPD, gyroPD, minSpeed, accLength, decLength, LFslowDistance
    global heading, yawReset, gyroRatio
    global lightTarget, lightHigh, lightLow, junctionThreshold
    global programStart
    global color
    global ref_heading

    lightPD = [12, 8000]
    gyroPD = [1300, 60000]
    minSpeed = 35
    accLength = 150
    decLength = 230
    LFslowDistance = 80

    observed = 106.5
    target = 90
    gyroRatio = 2 - observed/target
    lightHigh = 450
    lightLow = 60
    lightTarget = (lightHigh + lightLow) / 2 - 100
    junctionThreshold = lightLow + 100

    heading = 0
    yawReset = 0
    ref_heading = 0

    programStart = now()
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    wait(0.1)

# =============================================================================
#                            CORE I/O
# =============================================================================
def run(left, right):
    """Drives left and right motors with given speed (duty cycle: -100 to 100)."""
    motor.set_duty_cycle(l_wheel, round(left * -1))
    motor.set_duty_cycle(r_wheel, round(right))

def stop():
    """Stops both motors."""
    run(0, 0)

def now():
    """Returns current time in milliseconds."""
    return time.ticks_ms()

def wait(time_sec):
    """Busy-waits for specified seconds."""
    start = now()
    while now() - start <= time_sec * 1000:
        pass

def set_heading(degree):
    """Sets target heading for gyro-based movement."""
    global heading
    heading = degree

# =============================================================================
#                            SENSOR ROUTINES
# =============================================================================
def get_yaw():
    """Updates global yaw with corrected gyro reading (bounded to ±180)."""
    global yaw
    yaw = (motion_sensor.tilt_angles()[0] - yawReset) * -0.1
    yaw -= round((yaw - heading) / 361) * 360

def reset_yaw():
    """Soft-resets yaw tracking by recording new reference."""
    global yawReset, heading
    yawReset = motion_sensor.tilt_angles()[0]
    heading = 0

direction = {1:[-1, 1], -1:[1, -1] }

def reset_absolute():
    ref_heading = motion_sensor.tilt_angles()[0]
    return ref_heading

def head_up():
    delta = motion_sensor.tilt_angles()[0] - ref_heading
    flag = int(delta / abs(delta))

    lf, rf = direction[flag][0], direction[flag][1]
    print(delta)
    #turn (lf*60, rf*60, delta)

def get_light():
    """Reads reflected red value from floor color sensor."""
    global light
    light = color_sensor.rgbi(front_reader)

def ground_light():
    return color_sensor.rgbi(ground_reader)

# =============================================================================
#                        RESET ROUTINES / POSITION ROUTINES
# =============================================================================

def motor_reset():
    motor.reset_relative_position(l_wheel, 0)
    motor.reset_relative_position(r_wheel, 0)

def current_position():
    return (motor.relative_position(r_wheel) - motor.relative_position(l_wheel)) / 2

# =============================================================================
#                        MOTION ROUTINES
# =============================================================================
def move(speed, total_distance):
    """
    Gyro-stabilized straight drive.
    Args:
        speed: signed percentage (±100)
        total_distance: encoder degrees to travel
    """
    global power

    motor_reset()

    moved = 0
    set_pd(gyroPD)
    while moved <= total_distance:
        moved = (abs(motor.relative_position(l_wheel)) + abs(motor.relative_position(r_wheel))) / 2
        power = acc(moved, total_distance, speed) * 100
        power = round((abs(speed) / speed) * power)
        get_yaw()
        correction = round(pd(yaw, heading))
        run(power - correction, power + correction)
    stop()


# =============================================================================
#                            COLOR DETECTION
# =============================================================================


def move_cm(speed, cm):
    """
    Wrapper for move(). Converts cm to degrees, then calls move().
    Args:
        speed: signed percentage (±100)
        cm: distance in centimeters to travel
    """
    wait(0.5)
    reset_yaw()
    move(speed, distance_angle_finder(cm))
    stop()

def turn(left, right, angle_change):

    """
    Gyro-assisted turn using heading tracking.
    Args:
        left/right: motor speed (±100)
        angle_change: target angle to rotate (positive only)
    """
    global heading

    angle = int(angle_change * gyroRatio)

    heading += angle if left > right else -angle

    adjust = abs((left - right) / 100)

    get_yaw()

    remaining = abs(yaw - heading)

    while remaining >= 6 * adjust:

        get_yaw()
        remaining = abs(yaw - heading)
        if angle - remaining <= 6 * adjust:
            scale = 45
        elif angle - remaining <= 10 * adjust:
            scale = 70
        elif remaining > 30 * adjust:
            scale = 100
        elif remaining > 15 * adjust:
            scale = 60
        else:
            scale = 40
        run(left * scale, right * scale)

    stop()





async def stabilize():
    runloop.until(motion_sensor.stable)
    reset_yaw()
def junction_stop(side, speed):
    """
    Line-follow until junction is reached.
    Args:
        side: -1 for right, +1 for left (directional flag)
        speed: percentage of base speed
    """
    global power, heading
    set_pd(lightPD)
    power = speed * 100
    get_light()
    while light[0] > junctionThreshold:
        get_light()
        correction = pd(lightTarget, light[0]) * side
        run(power - correction, power + correction)
        get_light()
    stop()

def line_stop(speed):
    """
    Drives forward until black is detected, then white.
    Uses reflected light for white-black transition.
    """
    global power
    power = speed * 100
    set_pd(gyroPD)
    ground = ground_light()
    while ground[0] < lightTarget:
        get_yaw()
        correction = round(pd(yaw, heading))
        run(power - correction, power + correction)
        get_light()
    while color_sensor.color(ground_reader) != color.BLACK:
        get_yaw()
        correction = round(pd(yaw, heading))
        run(power - correction, power + correction)
        get_light()
    stop()


def line_stop_2(speed):
    """
    Drives forward until black is detected, then white.
    Uses reflected light for white-black transition.
    """
    global power
    power = speed * 100
    set_pd(gyroPD)
    ground = ground_light()
    while ground[0] < lightTarget:
        get_yaw()
        correction = round(pd(yaw, heading)/2)
        run(power - correction, power + correction)
        get_light()
    while color_sensor.color(ground_reader) != color.BLACK:
        get_yaw()
        correction = round(pd(yaw, heading))
        run(power - correction, power + correction)
        get_light()
    stop()

def follow(side, speed, total_distance):
    """
    Line-follow with acceleration ramp.
    Args:
        side: -1 = right sensor, +1 = left sensor (directional flag)
        speed: top speed
        total_distance: degrees to travel
    """
    global power, heading

    motor_reset()

    moved = 0
    set_pd(lightPD)
    while moved <= total_distance:
        moved = (abs(motor.relative_position(l_wheel)) + abs(motor.relative_position(r_wheel))) / 2
        slow_zone = max(0, moved - LFslowDistance)
        power = acc(slow_zone, total_distance - LFslowDistance, speed) * 100
        observed = ground_light()
        correction = pd(lightTarget, observed[0]) * side
        run(power - correction, power + correction)
    stop()
    reset_yaw()

# =============================================================================
#                        CALCULATION ROUTINES
# =============================================================================
def acc(moved, total, target_speed):
    """
    Calculates smoothed acceleration speed.
    """
    speed_range = abs(target_speed) - minSpeed
    if moved <= accLength:
        speed = speed_range * (moved / accLength) + minSpeed
    elif total - moved <= decLength:
        speed = speed_range * ((total - moved) / decLength) + minSpeed
    else:
        speed = abs(target_speed)
    return speed

def set_pd(pd_vals):
    """
    Sets PD constants.
    """
    global kP, kD, lastError
    kP = pd_vals[0]
    kD = pd_vals[1]
    lastError = 0

def pd(target, actual):
    """
    Returns PD output for given error input.
    """
    global lastError, PDscale, power
    PDscale = abs(power) / 10000
    error = target - actual
    result = (error * kP) + ((error - lastError) * kD)
    result *= PDscale
    lastError = error
    return result

def distance_angle_finder(distance):
    """
    Converts real-world distance (cm) to motor encoder degrees.
    Uses arc length = radius × angle, solved for angle.
    """
    return int(math.degrees(distance / wheel_radius))



# =============================================================================
#                                MISSION MOVEMENT
# =============================================================================


async def first_mission():
    await front_arm_movement(24, 1000)
    move_cm(-50,27.5)
    await front_arm_movement(24, 1000)
    await front_arm_movement(-24, 1000)
    wait(1)
    move_cm(75, 5)
    reset_yaw()
    turn(75, -75, 90)
    wait(1)
    reset_yaw()
    move_cm(-75, 10)
    turn(-75, 75, 90)
    reset_yaw()
    move_cm(-100, 30)
    await front_arm_movement(24, 1000)
async def mission_two():
    move_cm(100, 21.7)
    await front_arm_movement(110, 1000)
    wait(0.5)
    motor.stop(l_wheel, stop = motor.HOLD)
    motor.stop(r_wheel, stop = motor.HOLD)
    await front_arm_movement(-110, 400)
    await front_arm_movement(110, 1000)
    await front_arm_movement(-110, 400)
    await front_arm_movement(110, 1000)
    move_cm(-100, 50)



# =============================================================================
#                        Mission ROUTINES
# =============================================================================




async def main():
    initialize()
    setup()
    # await first_mission()
    await front_arm_movement(-22, 1000)
    move_cm(75, 41)
    move_cm(-100, 5)
    turn(-100,100, 16.5)
    
    await front_arm_movement(4, 500)
    move_cm(-75, 2)
    move_cm(50, 3)
    wait(1)
    motor.stop(front_arm, stop = motor.HOLD)
    turn(-100,100, 45.5)

    move_cm(50, 1)
    # turn(75,0, 50)
    # turn(0,75, 50)

    









runloop.run(main())
