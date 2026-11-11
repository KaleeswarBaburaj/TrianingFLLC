from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction, Color, Stop
from pybricks.tools import wait, StopWatch

class MissionRobot:
    def __init__(self):
        # Initialize the Hub
        self.hub = PrimeHub()
    
        # Initialize Drive Motors (Adjust ports and directions as needed)
        self.left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B, Direction.CLOCKWISE)
        
        # Initialize DriveBase (Wheel diameter: 56mm, Axle track/width: 124mm)
        # Change these dimensions to match your specific robot!
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter=56, axle_track=115)
        self.drive_base.use_gyro(True)
        # Set speed and turn rate
        #self.drive_base.settings(straight_speed=500, straight_acceleration=400, turn_rate=90, turn_acceleration= 100)

        # Initialize Attachment Motors
        self.left_attachment = Motor(Port.C)
        self.right_attachment = Motor(Port.D)
        
        # Initialize Sensors
        self.color_sensor = ColorSensor(Port.E)
        self.distancesensor = UltrasonicSensor(Port.F)

        self.heading_pid = PID(kp=2.5, ki=0.02, kd=8)
        self.turn_pid = PID(kp=2.5, ki=0.0, kd=15) 


    # Move Robot 
    def reset_heading(self):
        self.hub.imu.reset_heading(0)

    def gyro_straight(self, distance_mm, speed=300):
        self.heading_pid.reset()
        self.drive_base.reset()
        self.turn_pid.reset()
        
        while abs(self.drive_base.distance()) < abs(distance_mm):
            heading_error = 0 - self.hub.imu.heading()
            correction = self.heading_pid.compute(heading_error)
            self.drive_base.drive(speed, correction)
        self.drive_base.stop()

    def line_follow(self, distance_mm, speed=150, target_reflection=50):
        """
        Follow a line edge using the light sensor's reflection value.
        target_reflection is the value halfway between the line color
        and the background — e.g. ~50 if black line (~10) and white
        background (~90) average to 50. Calibrate this for your mat.
        """
        self.line_pid.reset()
        self.drive_base.reset()

        while abs(self.drive_base.distance()) < abs(distance_mm):
            reflection = self.color_sensor.reflection()
            error = target_reflection - reflection
            correction = self.line_pid.compute(error)
            self.drive_base.drive(speed, correction)

        self.drive_base.stop()

    def gyro_turn(self, target_angle, timeout_ms=3000, tolerance=1):
        """
        Turns the robot in place to an absolute heading using the gyro.
        target_angle: degrees, relative to the last reset_heading() call.
                      Positive = clockwise (right), negative = counter-
                      clockwise (left) -- matches Pybricks' heading convention.
        tolerance: how close (in degrees) counts as "done".
        """
        self.turn_pid.reset()
        timer = StopWatch()
        timer.reset()

        while timer.time() < timeout_ms:
            current_heading = self.hub.imu.heading()
            error = target_angle - current_heading

            if abs(error) <= tolerance:
                break

            turn_rate = self.turn_pid.compute(error)
            self.drive_base.drive(0, turn_rate)  # 0 speed = rotate in place

        self.drive_base.stop()

    def gyro_turn_right(self, degrees, timeout_ms=3000):
        """Turns clockwise (right) by a relative amount from current heading."""
        current_heading = self.hub.imu.heading()
        self.gyro_turn(current_heading + abs(degrees), timeout_ms)

    def gyro_turn_left(self, degrees, timeout_ms=3000):
        """Turns counter-clockwise (left) by a relative amount from current heading."""
        current_heading = self.hub.imu.heading()
        self.gyro_turn(current_heading - abs(degrees), timeout_ms)


    def move_robot(self,distance=0, speed = 0 , accl = 0 ):
        # Set speed and turn rate
        self.drive_base.settings(straight_speed=speed, straight_acceleration=accl, turn_rate=90, turn_acceleration= 100)
        self.drive_base.straight(distance)
        
    # --- ATTACHMENT MOTOR FUNCTIONS ---
    
    def run_left_attachment(self,  angle, speed = 500, wait=True):
        #"""Runs the left attachment motor to a specific angle."""
        print("Running left attachment...")
        self.left_attachment.reset_angle(0)
        self.left_attachment.run_target(speed, angle,Stop.HOLD,wait)
        #self.left_attachment.run(speed)
        #self.drive_base.straight(300, then=Stop.BRAKE)
        print("stop running attachment")
        
    def run_right_attachment(self, angle, speed = 500, wait=True):
        #"""Runs the right attachment motor to a specific angle."""
        print("Running right attachment...")
        self.right_attachment.reset_angle(0)
        self.right_attachment.run_target(speed, angle,Stop.HOLD, wait)

    # # --- SENSOR FUNCTIONS ---
    # def stop_at_black_line(self, sensor_port_motor="line_sensor", speed=200,
    #                         black_threshold=15, timeout_ms=5000,mode="either"):
    #     """
    #     Drives straight until the sensor detects black, then stops.
    #     black_threshold: reflection value below this counts as "black".
    #                       Calibrate this for your mat (black is usually 0-20).
    #     """
    #     self.heading_pid.reset()
    #     timer = StopWatch()
    #     timer.reset()
    #     while timer.time() < timeout_ms:
    #         reflection_1 = self.color_sensor.reflection()
    #         reflection_2 = self.color_sensor2.reflection()

    #         sees_black_1 = reflection_1 <= black_threshold
    #         sees_black_2 = reflection_2 <= black_threshold

    #         if mode == "either" and (sees_black_1 or sees_black_2):
    #             break
    #         if mode == "both" and (sees_black_1 and sees_black_2):
    #             break

    #         heading_error = 0 - self.hub.imu.heading()
    #         correction = self.heading_pid.compute(heading_error)
    #         self.drive_base.drive(speed, correction)

        # while timer.time() < timeout_ms:
        #     reflection = self.line_sensor.reflection()

        #     if reflection <= black_threshold:
        #         break

        #     heading_error = 0 - self.hub.imu.heading()
        #     correction = self.heading_pid.compute(heading_error)
        #     self.drive_base.drive(speed, correction)
        # print("Black line detected. Stopped.")
        # self.drive_base.stop()
    # def drive_until_black_line(self, speed):
    #     #"""Drives forward until the color sensor detects a black line."""
    #     print("Driving until black line...")
    #     self.drive_base.drive(speed, 0) # Drive straight at the given speed
        
    #     # Loop until reflection drops below 15% (typical for a black line)
    #     while self.color_sensor.reflection() > 15:
    #         wait(10)
            
    #     self.drive_base.stop()
    #     print("Black line detected. Stopped.")

    def avoid_obstacle(self, trigger_distance_mm=150):
        # """
        # Checks if an object is closer than the trigger distance.
        # If so, backs up and turns left.
        # """
        distance = self.distance_sensor.distance()
        print("Current distance:", distance, "mm")
        
        if distance <= trigger_distance_mm:
            print("Object detected! Executing avoidance maneuver...")
            # Move straight backward for 150 mm
            self.drive_base.straight(-150)
            # Turn left 90 degrees
            self.drive_base.turn(-90)
            print("Maneuver complete.")
        else:
            print("Path is clear.")


class PID:
    def __init__(self, kp, ki, kd, integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit  # prevents integral windup

        self.integral = 0
        self.last_error = 0
        self.timer = StopWatch()
        self.last_time = 0

    def reset(self):
        self.integral = 0
        self.last_error = 0
        self.timer.reset()
        self.last_time = 0

    def compute(self, error):
        current_time = self.timer.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 1  # avoid divide-by-zero on the very first call

        self.integral += error * dt
        # clamp integral so it can't wind up out of control
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        derivative = (error - self.last_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.last_error = error
        self.last_time = current_time

        return output

# Port conflict check: you now have right_attachment on Port C and left_attachment 
# on Port D — make sure no other mission file (or the earlier arm_motor) is also trying to claim C or D, 
# or you'll hit that EBUSY error again.

# wait=False for parallel actions: if a mission needs the robot to drive and run an attachment at the same time 
# (common for scoring on the move), call run_right_attachment(angle, wait=False) before the drive command — 
# otherwise the attachment finishes fully before the robot moves at all.

# Calibrate black_threshold the same way as target_reflection earlier — print robot.line_sensor.reflection() live 
# while holding the sensor over black vs. white to get real numbers for your mat and lighting.

# PID broken down simply (Read through it to understand)

# PID = Proportional + Integral + Derivative — three ways of looking at an error (the difference between where you want to be and where you are), combined to decide how hard to correct.

# Think of driving a car and trying to stay centered in a lane:

# P (Proportional) — "How far off am I right now?"

# If you're way off center, steer hard. If you're just slightly off, steer gently.
# Correction = error × some constant (Kp)
# Problem alone: you tend to overshoot and oscillate back and forth, or you never quite reach the target 
#(settles with a small leftover error).

# I (Integral) — "How long have I been off, and by how much accumulated?"

# If you've been drifting slightly left for a while, even a small consistent error adds up. The I term notices "hey, this has been off for a while" and adds extra push to eliminate that lingering error.
# Correction = sum of all past errors × constant (Ki)
# Fixes the "never quite gets there" problem of P alone, but too much I makes things overshoot and oscillate (it can "overcorrect" from built-up history).

# D (Derivative) — "How fast am I approaching/leaving the target?"

# If you're rushing toward center really fast, ease off the correction so you don't blow past it. It's a brake on sudden changes.
# Correction = rate of change of error × constant (Kd)
# Adds damping — smooths things out and reduces overshoot/oscillation.

# Put together:

# correction = Kp*(error) + Ki*(sum of past errors) + Kd*(rate of change of error)

# Analogy: Imagine parking a car by looking at a target line.

# P: "I'm 3 feet off, turn the wheel proportionally."
# I: "I've been drifting off for the last 10 seconds despite small corrections — add more oomph."
# D: "I'm approaching the line really fast — ease up so I don't overshoot."