from pololu_3pi_2040_robot import robot
import time


class MOVE_OPTIONS:

    ROTATE = 0
    LINEAR = 1

class MotionController:
    
    def __init__(self, motors, encoders, imu):
        self.motors = motors
        self.encoders = encoders
        self.imu = imu
        self.rotate_max_speed = 4000
        self.rotate_kp = 140
        self.rotate_kd = 4
        self.last_time_gyro_reading = None
        self.turn_rate = 0.0     # degrees per second
        self.robot_angle = 0.0   # degrees
        self.target_angle = 0.0
        self.last_time_far_from_target = None
        self.debug = 0


    def move(self, mm):
        degree = self._convertDistanceToDegree(mm)
        # 1 encoder count == 2 degree
        count = int (degree/2)

        turnSpeed = 700
        if mm < 0:
            turnSpeed = -turnSpeed

        originalAngle = self._getRobotAngle()
        counts = self.encoders.get_counts(reset = True)

        compensateLeft = 0
        compensateRight = 0
        while abs(counts[0])<abs(count) or abs(counts[1])<abs(count):
        
            # compensate rotation and stepper count
            newAngle = self._getRobotAngle()
            diffAngle = int(newAngle - originalAngle)
            self.debug = diffAngle
            
            turn_speed = diffAngle * self.rotate_kp - self.turn_rate * self.rotate_k
            
            if diffAngle < 0:
                compensateLeft += turn_speed
                compensateRight = 0
            elif diffAngle > 0:
                compensateLeft = 0
                compensateRight += turn_speed
            else:
                compensateLeft = 0
                compensateRight = 0
            leftSpeed = turnSpeed+compensateLeft
            rightSpeed = turnSpeed+compensateRight
            self.motors.set_speeds(leftSpeed, rightSpeed)


    def stop(self):
        self.motors.off()


    def rotate(self, degree):
        currentAngle = self._getRobotAngle()
        targetAngle = currentAngle + degree
        
        while True:
            self._getRobotAngle()
            far_from_target = abs(self.robot_angle - targetAngle) > 3
            if far_from_target:
                self.last_time_far_from_target = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), self.last_time_far_from_target) > 250:
                # self.motors.off()
                break

            turn_speed = (targetAngle - self.robot_angle) * self.rotate_kp - self.turn_rate * self.rotate_kd
            if turn_speed > self.rotate_max_speed: turn_speed = self.rotate_max_speed
            if turn_speed < -self.rotate_max_speed: turn_speed = -self.rotate_max_speed
            self.motors.set_speeds(-turn_speed, turn_speed)

            self.turn_speed = turn_speed


    def _getRobotAngle(self):
        while not self.imu.gyro.data_ready():
            continue

        self.imu.gyro.read()
        self.turn_rate = self.imu.gyro.last_reading_dps[2]  # degrees per second
        now = time.ticks_us()
        if self.last_time_gyro_reading:
            dt = time.ticks_diff(now, self.last_time_gyro_reading)
            self.robot_angle += self.turn_rate * dt / 1000000
        self.last_time_gyro_reading = now
        
        return self.robot_angle


    def _convertDistanceToDegree(self, mm):
        # wheel d = 32mm
        return int( mm * 360 / (32 * 3.14) )


class Robot:
    def __init__(self):
        self.display = robot.Display()

        self.display.fill(0)
        self.display.text("Starting IMU...", 0, 0, 1)
        self.display.show()

        self.motors = robot.Motors()
        self.encoders = robot.Encoders()
        self.imu = robot.IMU()
        self.imu.reset()
        self.imu.enable_default()
        self.button_a = robot.ButtonA()
        self.button_b = robot.ButtonB()
        self.button_c = robot.ButtonC()

        self.motionController = MotionController(self.motors, self.encoders, self.imu)

        self.display.text("DONE", 0, 10, 1)
        self.display.show()


r = Robot()
angle = 0
step = 10

def draw_text():
    global r, step, counter
    r.display.fill(0)
    r.display.text(f"A: Turn left {step} deg", 0, 0, 1)
    r.display.text(f"C: Turn right {step} deg", 0, 8, 1)
    r.display.text(f"target: {angle}", 0, 16, 1)
    r.display.text(f"debug: {r.motionController.debug}", 0, 24, 1)
    r.display.text(f"Angle", 0, 32, 1)
    r.display.show()


while True:
    draw_text()

    if r.button_a.check() == True:
        angle -= step
    if r.button_c.check() == True:
        angle += step
    if r.button_b.check() == True:
        time.sleep_ms(500)
        r.motionController.rotate(-angle)
        # r.motionController.move(angle)
        angle = 0
    # for _ in range(0, 1000):
    # r.motionController.move(1000)
    # for _ in range(0,4):
    #     r.motionController.move(50)
    #     r.motionController.rotate(90)


# have display controller
# do while check button a not pressed & released
# add logger
# pid from line follower

# while not button_a.check():
#     pass

# display.fill(0)
# display.show()
# time.sleep(.5)