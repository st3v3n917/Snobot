import serial
import pynmea2

"""
gpsTest.py
just print lat, lon gpsData 
@authors: Chanyang Yim
"""

ser = serial.Serial('/dev/ttyACM1',baudrate=9600,timeout=1)

while True:
    msg = ser.readline().decode('utf-8', 'ignore')
    #print(msg)
    if msg.startswith('$'):
        if "GGA" in msg:
            gpsCoord = pynmea2.parse(msg)
            #print(gpsCoord.longitude)
            print("[",gpsCoord.latitude,",",gpsCoord.longitude,"]")
        #print(ser.readline())
