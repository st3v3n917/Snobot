import serial
import pynmea2
import time, inspect
import csv
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055
import busio
import board

filename = 'GPSData.csv'

ser = serial.Serial('/dev/ttyACM0',baudrate=9600,timeout=1)
gpsData = open(filename, 'w')
fieldnames = ['latitude','longitude']
gps = csv.DictWriter(gpsData, fieldnames=fieldnames, delimiter = ' ')
gps.writeheader()



# Create library object using our Extended Bus I2C port
# Use `ls /dev/i2c*` to find out what i2c devices are connected
i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Device is /dev/i2c-1
sensor = adafruit_bno055.BNO055_I2C(i2c)


try:
    while True:
        msg = ser.readline().decode('utf-8', 'ignore')
        if msg.startswith('$'):
            if "GGA" in msg:
                gpsCoord = pynmea2.parse(msg)
                latitude = gpsCoord.latitude
                longitude = gpsCoord.longitude
                #print("{} {}".format(latitude,longitude))
                time.sleep(5)
               # gps.writeheader()
                gps.writerow({'latitude' : latitude, 'longitude' : longitude})

except (KeyboardInterrupt, SystemExit):
    print("Done.\nExiting.")
    gpsData.close()
