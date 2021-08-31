#Libraries
import time
from adafruit_servokit import ServoKit
import curses
import adafruit_bno055
import board
import pynmea2
import serial
from math import sin, cos, sqrt, atan2, radians
import numpy
import pynmea2
import inspect
import csv
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055
import busio

#Constants
PCAServo=16 #Total of 16 Channels
MIN_PWM=1100 #Minimum PWM Signal Sent
MAX_PWM=1900 #Max PWM Signal Sent

#Parameters
Brake = 70 #Stop Servos from Moving
Max_Speed = 70 #Max Speed Servos can turn

#Motor Channels
LeftMotor = 2
RightMotor = 1
Blower = 0

#Motor Driver Objects
pca = ServoKit(channels=16)
stdscr = curses.initscr()

filename = 'GPSData.csv'

#ser = serial.Serial('/dev/ttyACM1',baudrate=9600,timeout=1)
#gpsData = open(filename, 'w')
#fieldnames = ['latitude','longitude','angle']
#gps = csv.DictWriter(gpsData, fieldnames=fieldnames, delimiter = ' ')
#gps.writeheader()

# Create library object using our Extended Bus I2C port
# Use `ls /dev/i2c*` to find out what i2c devices are connected
#i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Device is /dev/i2c-1
#sensor = adafruit_bno055.BNO055_I2C(i2c)

def main():
    Manual()

def init():
    for i in range(PCAServo):
        pca.servo[i].set_pulse_width_range(MIN_PWM, MAX_PWM)
        pca.servo[i].angle = Brake

#function Manual
#Move Snobot with Keyboard only
def Manual():
    try:
        while True:
            key = stdscr.getch()
            #Forward
            if key == ord('w'):
                forward()
               # record()
            #Move Left
            elif key == ord('a'):
                left()
               # record()
            #Move Right
            elif key == ord('d'):
                right()
             #   record()
            #Reverse
            elif key == ord('s'):
                reverse()
              #  record()
            elif key == ord('e'):
                brake()
               # record()
                curses.endwin()
    #Stop all motors you stop program
    except KeyboardInterrupt:
        print("end")
        brake()
        gpsData.close()
        curses.endwin()

def forward(): #0.1194 m/s
    pca.servo[LeftMotor].angle = 97       #forward above 80 reverse below 15
    pca.servo[RightMotor].angle = 50 #forward below 50 reverse above 50
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def left():
    pca.servo[LeftMotor].angle = 50
    pca.servo[RightMotor].angle = 50
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def right():
    pca.servo[LeftMotor].angle = 95
    pca.servo[RightMotor].angle = 95
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def reverse():
    pca.servo[LeftMotor].angle = 50
    pca.servo[RightMotor].angle = 95
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def brake():
    pca.servo[LeftMotor].angle = Brake
    pca.servo[RightMotor].angle = Brake
    pca.servo[Blower].angle = Brake
    time.sleep(0.05)

#def record():
 #   msg = ser.readline().decode('utf-8', 'ignore')
  #  if msg.startswith('$'):
   #     if "GGA" in msg:
    #        gpsCoord = pynmea2.parse(msg)
     #       latitude = gpsCoord.latitude
      #      longitude = gpsCoord.longitude
            #print("{} {}".format(latitude,longitude))
       #     angle = sensor.euler[0]
        #    time.sleep(5)
            # gps.writeheader()
         #   gps.writerow({'latitude' : latitude, 'longitude' : longitude, 'angle' : angle})


if __name__ == '__main__':
    init()
    main()

