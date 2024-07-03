import time
from machine import I2C, Pin
from vl6180x import Sensor
from pcf8574 import PCF8574
from pololu_3pi_2040_robot import robot

display = robot.Display()
motors = robot.Motors()
motors.flip_left(False)
motors.flip_right(False)

i2c = I2C(id=0, scl=Pin(5), sda=Pin(4), freq=100_000)

pcf = PCF8574(i2c, 0x20)

pcf.port = 0x00
pcf.pin(0, 1)
pcf.pin(1, 0)
pcf.pin(2, 0)
pcf.pin(3, 0)
pcf.pin(4, 0)

display.fill(0)
display.text("PCF init", 0, 0)
display.show()
time.sleep_ms(100)

tof1 = Sensor(i2c,0x29)
tof1.address(0x2A)

display.fill(0)
display.text("TOF1 init", 0, 0)
display.show()
time.sleep_ms(100)


pcf.pin(1, 1)
time.sleep_ms(100)

tof2 = Sensor(i2c,0x29)
tof2.address(0x2B)

display.fill(0)
display.text("TOF2 init", 0, 0)
display.show()
time.sleep_ms(100)

pcf.pin(2, 1)
time.sleep_ms(100)

tof3 = Sensor(i2c,0x29)
tof3.address(0x2C)
time.sleep_ms(100)
display.fill(0)
display.text("TOF3 init", 0, 0)
display.show()
time.sleep_ms(100)

pcf.pin(3, 1)
time.sleep_ms(100)


tof4 = Sensor(i2c,0x29)
tof4.address(0x2E)
time.sleep_ms(100)
display.fill(0)
display.text("TOF4 init", 0, 0)
display.show()
time.sleep_ms(100)

pcf.pin(4, 1)
time.sleep_ms(100)

tof5 = Sensor(i2c,0x29)
tof5.address(0x2F)
time.sleep_ms(100)
display.fill(0)
display.text("TOF5 init", 0, 0)
display.show()
time.sleep_ms(100)

display.fill(0)
display.text("Start ranging", 0, 0)
display.show()
time.sleep_ms(500)


tof1.startContRange()
tof2.startContRange()
tof3.startContRange()
tof4.startContRange()
tof5.startContRange()

display.fill(0)
display.text("Ranging on", 0, 0)
display.show()
time.sleep_ms(500)

distance1, distance2, distance3, distance4, distance5 = 0, 0, 0, 0, 0

kp = 0.3
ki = 0.0
kd = 0.05

t, last_t = 0.0, 0.0
e=0
last_e=0
base_power = 600
max_power = 2000

p, i, d = 0, 0, 0

def update_display(left_motor,right_motor,pid,d1,d5):
  display.fill(0)
  display.text("left_motor: " + str(left_motor), 0, 0)
  display.text("right_motor: " + str(right_motor), 0, 10)
  display.text("pid: " + str(pid), 0, 20)
  display.text("distance1: " + str(d1), 0, 30)
  display.text("distance2: " + str(d5), 0, 40)
  display.show()

num_it = 0
update_display(0,0,0,0,0)
time.sleep_ms(500)

display.fill(0)
display.text("Starting main loop", 0, 0)
display.show()
time.sleep_ms(500)

while True:


  distance1 = tof1.readContRange()
  distance2 = tof2.readContRange()
  distance3 = tof3.readContRange()
  distance4 = tof4.readContRange()
  distance5 = tof5.readContRange()

  t = time.ticks_us()

  e = distance1 + distance2 - distance5 - distance4

  p = kp * e
  i = 0#i + ki * (e*(t-last_t))
  d = kd * ((e-last_e)/(t-last_t))

  pid = p + i + d

  left_motor = base_power + pid
  right_motor = base_power - pid

  motors.set_speeds(left_motor,right_motor)

  last_t = t
  last_e = e

  num_it += 1

  if num_it > 100:
    update_display(left_motor,right_motor,pid,distance1,distance5)
    num_it = 0

  #display.fill(0)
  #display.text("NumIt: " + str(num_it), 0, 0)
  #display.show()
  #time.sleep_ms(500)



