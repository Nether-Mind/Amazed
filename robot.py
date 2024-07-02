from pololu_3pi_2040_robot import robot
import

class MotionController:
    def __init__(self, motors, encoders, imu):
        self.motors = motors
        self.encoders = encoders
        self.imu = imu
        self.rotate_max_speed = 3000
        self.rotate_kp = 140
        self.rotate_kd = 4
        self.last_time_gyro_reading = None
        self.turn_rate = 0.0     # degrees per second
        self.robot_angle = 0.0   # degrees
        self.target_angle = 0.0
        self.last_time_far_from_target = None


    def turnLeft(self, degree):
        self._doRotate(degree)


    def turnRight(self, degree):
        self._doRotate(-degree)


    def _getRobotAngle(self):
        while not imu.gyro.data_ready():
            continue

        self.imu.gyro.read()
        self.turn_rate = imu.gyro.last_reading_dps[2]  # degrees per second
        now = time.ticks_us()
        if self.last_time_gyro_reading:
            dt = time.ticks_diff(now, self.last_time_gyro_reading)
            self.robot_angle += self.turn_rate * dt / 1000000
        self.last_time_gyro_reading = now
        
        return self.robot_angle


    def _doRotate(self, degree):
        currentAngle = self._getRobotAngle()
        targetAngle = currentAngle + degree
        
        while True:
            self._getRobotAngle()
            far_from_target = abs(self.robot_angle - targetAngle) > 3
            if self.far_from_target:
                self.last_time_far_from_target = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), self.last_time_far_from_target) > 250:
                self.motors.off()
                break

            turn_speed = (targetAngle - self.robot_angle) * kp - turn_rate * kd
            if turn_speed > max_speed: turn_speed = max_speed
            if turn_speed < -max_speed: turn_speed = -max_speed
            motors.set_speeds(-turn_speed, turn_speed)


class Robot:
    def __init__(self):
        self.motors = robot.Motors()
        self.encoders = robot.Encoders()
        self.imu = robot.IMU()
        self.imu.reset()
        self.imu.enable_default()

        self.motionController = MotionControl(self, motors, self.encoders, self.imu)