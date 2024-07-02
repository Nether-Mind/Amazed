import time
from machine import I2C, Pin
from vl6180x import Sensor
from pcf8574 import PCF8574
from pololu_3pi_2040_robot import robot

display = robot.Display()


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

tof1.startContRange()
tof2.startContRange()
tof3.startContRange()
tof4.startContRange()
tof5.startContRange()

while True:

  distance1 = tof1.readContRange()
  distance2 = tof2.readContRange()
  distance3 = tof3.readContRange()
  distance4 = tof4.readContRange()
  distance5 = tof5.readContRange()
  display.fill(0)
  display.fill_rect(59, 47, 10, 10, 1)
  display.text(str(distance5), 6, 47)
  display.line(33, 50, 55, 50, 1)
  display.text(str(distance4), 15, 25)
  display.line(27, 35, 58, 46, 1)
  display.text(str(distance3), 52, 10)
  display.line(63, 21, 63, 41, 1)
  display.text(str(distance2), 85, 25)
  display.line(97, 35, 68, 46, 1)
  display.text(str(distance1), 100, 47)
  display.line(73, 50, 95, 50, 1)
  display.show()
  
  

  
