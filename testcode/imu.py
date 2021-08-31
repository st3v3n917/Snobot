import time
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055
import busio
import board

# To enable i2c-gpio, add the line `dtoverlay=i2c-gpio` to /boot/config.txt
# Then reboot the pi

# Create library object using our Extended Bus I2C port
# Use `ls /dev/i2c*` to find out what i2c devices are connected
i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Device is /dev/i2c-1
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.COMPASS_MODE
last_val = 0xFFFF

def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result


while True:
    print("Temperature: {} degrees C".format(temperature()))
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle (Heading): {}".format(sensor.euler[0]))
    print("Euler angle (Roll): {}".format(sensor.euler[1]))
    print("Euler angle (Ptch): {}".format(sensor.euler[2]))
    print("Quaternion Angle (Heading): {}".format(sensor.quaternion[0]))
    print("Quaternion Angle (Heading): {}".format(sensor.quaternion[1]))
    print("Quaternion Angle (Heading): {}".format(sensor.quaternion[2]))
#    print("Quaternion Angle (Heading): {}".format(sensor.quaternion[3]))
    print()

    time.sleep(1)
