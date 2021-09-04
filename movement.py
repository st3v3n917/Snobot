#Libraries
import time
from adafruit_servokit import ServoKit
import curses
import adafruit_bno055
from adafruit_extended_bus import ExtendedI2C as I2C
import busio
import board
import pynmea2
import serial
from math import sin, cos, sqrt, atan2, radians, degrees
import numpy as np
import csv
import inspect
import pyrealsense2.pyrealsense2 as rs
import cv2

#Constants
PCAServo=16 #Total of 16 Channels
MIN_PWM=1100 #Minimum PWM Signal Sent
MAX_PWM=1900 #Max PWM Signal Sent

#Parameters
Brake = 70 #Stop Servos from Moving
Max_Speed = 100 #Max Speed Servos can turn

#Motor Channels
LeftMotor = 2
RightMotor = 1
Blower = 0

#Motor Driver Objects
pca = ServoKit(channels=16)
stdscr = curses.initscr()

#GPS Objects
ser = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600, timeout = 1.0)
#er2 = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600, timeout = 1.0)

#IMU Objects
i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Device is /dev/i2c-1
sensor = adafruit_bno055.BNO055_I2C(i2c)

#Camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

#function init
def init():
    print("initializing...\r\n")
    #Motor Initializiation
    for i in range(PCAServo):
        pca.servo[i].set_pulse_width_range(MIN_PWM, MAX_PWM)
        pca.servo[i].angle = Brake
#function main
def main():
   auto()
    # print("What mode do you want to run?\r\n")
   # print("'auto' for autonomous movement or 'manual' for manual control\r\n")
   # print("in auto, the rover will move according to predefined path\r\n")
   # print("in manual, the rover will move according to your controls and will record the path\r\n")
   # input1 = input()
   # if input1 == "auto":    
   #     print("starting auto\r\n")
   #     auto()
   # elif input1 == "manual":
   #     print("starting manual\r\n")
   #     Thread(target = manual).start()
   #     Thread(target = gpswrite).start()
   # else:
   #     print("did you enter the wrong command?\r\n")
   #     print("try again\r\n")
    
######################NAVIGATION CODE#####################################################################

#poll for current GPS Coordinates
def gps():
    while True:
        msg = ser.readline().decode('utf-8', 'ignore')
        if msg.startswith('$'):
            if "GGA" in msg:
                gpsCoord = pynmea2.parse(msg)
                current_lat = gpsCoord.latitude
                current_lon = gpsCoord.longitude
                return current_lat, current_lon

#poll for current GPS Coordinates
def gps2():
    while True:
        msg = ser2.readline().decode('utf-8', 'ignore')
        if msg.startswith('$'):
            if "GGA" in msg:
                gpsCoord = pynmea2.parse(msg)
                current_lat = gpsCoord.latitude
                current_lon = gpsCoord.longitude
                return current_lat, current_lon


#Store GPS data from RTK GPS (C099-F9P)
def gpswrite():
    filename = 'GPSData.csv'
    gpsData = open(filename, 'w')
    fieldnames = ['latitude','longitude']
    gps = csv.DictWriter(gpsData, fieldnames=fieldnames, delimiter = ' ')
    gps.writeheader()

    while True:
        msg = ser.readline().decode('utf-8', 'ignore')
        #Exclude Non-GPS Related Values
        if msg.startswith('$'):
            #Acquire $GPGGA Data for Lat Lon Coord
            if "GGA" in msg:
                gpsCoord = pynmea2.parse(msg)
                latitude = gpsCoord.latitude
                longitude = gpsCoord.longitude
                #Frequency of Data Acquisition
                time.sleep(0.1)
                gps.writerow({'latitude' : latitude, 'longitude' : longitude})

#get GPS Data stored in GPSData.csv
def gpsread(count):
    filename = 'GPSData.csv'
    #Create Empty Set to store GPS Data from .csv file
    lat = []
    lon = []

    with open(filename) as gps:
        gpsreader = csv.DictReader(gps, delimiter = ' ')
        for row in gpsreader:
            #Convert str into float
            latitude = float(row['latitude'])
            longitude = float(row['longitude'])
            #Put Data in .csv file into an array
            lat.append(latitude)
            lon.append(longitude)
        desired_lat = lat[count]
        desired_lon = lon[count]
        return desired_lat, desired_lon

def csvlen():
    filename = 'GPSData.csv'
    num_row = 0
    with open(filename) as gps:
        gpsreader = csv.DictReader(gps)
        for row in gpsreader:
            num_row += 1
        return num_row

#determine bearing to know what angle the rover needs to turn to
def navigate(current_lat, current_lon, desired_lat, desired_lon):
    #convert lat and lon angles to radians
    lat1 = radians(current_lat)
    lon1 = radians(current_lon)
    lat2 = radians(desired_lat)
    lon2 = radians(desired_lon)
    
    #get change in lat and lon values
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    
    print("calculating bearing\r\n")
    #math for calculating bearing
    x = cos(lat2) * sin(dlon)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = np.arctan2(x,y)
    bearing = np.degrees(bearing)
    return bearing

#################################OBJECT AVOIDANCE CODE##################################

#function camera depth
def camera():
    #start streaming
#    pipeline.start(config)
    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    for frame in frames:
        if frame.is_depth_frame():
            depth = frames.get_depth_frame()
            for y in range(0,480,40):
                for x in range(0,600,40):
                    dist = depth.get_distance(x,y)
                    if y == 480:
                        pipeline.stop()
                        return
                    if dist > 0.1 and dist < 1:
                        if x > 200 and x < 400:
                            #center of camera, what to do?
                            hard_brake()
                            print("object in the way...\r\n")
                            pipeline.stop()
                            camera()
                        elif x < 200:
                            #left of camera, what to do?
                            print("object on left\r\n")
                            right()
                            time.sleep(0.3)
                            hard_brake()
                            pipeline.stop()
                            camera()
                        elif x > 400:
                            #right of camera, what to do?
                            print("object on right\r\n")
                            left()
                            time.sleep(0.3)
                            hard_brake()
                            pipeline.stop()
                            camera()
        pipeline.stop()
        return

##############EXTEDNDED KALMAN FILTER CODE#########################################

# A matrix
# 3x3 matrix -> number of states x number of states matrix
# Expresses how the state of the system [x,y,yaw] changes 
# from k-1 to k when no control command is executed.
# Typically a robot on wheels only drives when the wheels are told to turn.
# For this case, A is the identity matrix.
# A is sometimes F in the literature.
A_k_minus_1 = np.array([[1.0,  0,   0],
                        [  0,1.0,   0],
                        [  0,  0, 1.0]])
 
# Noise applied to the forward kinematics (calculation
# of the estimated state at time k from the state
# transition model of the mobile robot). This is a vector
# with the number of elements equal to the number of states
process_noise_v_k_minus_1 = np.array([0.01,0.01,0.003])
     
# State model noise covariance matrix Q_k
# When Q is large, the Kalman Filter tracks large changes in 
# the sensor measurements more closely than for smaller Q.
# Q is a square matrix that has the same number of rows as states.
Q_k = np.array([[1.0,   0,   0],
                [  0, 1.0,   0],
                [  0,   0, 1.0]])
                 
# Measurement matrix H_k
# Used to convert the predicted state estimate at time k
# into predicted sensor measurements at time k.
# In this case, H will be the identity matrix since the 
# estimated state maps directly to state measurements from the 
# odometry data [x, y, yaw]
# H has the same number of rows as sensor measurements
# and same number of columns as states.
H_k = np.array([[1.0,  0,   0],
                [  0,1.0,   0],
                [  0,  0, 1.0]])
                         
# Sensor measurement noise covariance matrix R_k
# Has the same number of rows and columns as sensor measurements.
# If we are sure about the measurements, R will be near zero.
R_k = np.array([[1.0,   0,    0],
                [  0, 1.0,    0],
                [  0,    0, 1.0]])  
                 
# Sensor noise. This is a vector with the
# number of elements equal to the number of sensor measurements.
sensor_noise_w_k = np.array([0.07,0.07,0.04])

def getB(yaw, deltak): 
    """
    Calculates and returns the B matrix
    3x2 matix -> number of states x number of control inputs
    The control inputs are the forward speed and the
    rotation rate around the z axis from the x-axis in the 
    counterclockwise direction.
    [v,yaw_rate]
    Expresses how the state of the system [x,y,yaw] changes
    from k-1 to k due to the control commands (i.e. control input).
    :param yaw: The yaw angle (rotation angle around the z axis) in rad 
    :param deltak: The change in time from time step k-1 to k in sec
    """
    B = np.array([  [np.cos(yaw)*deltak, 0],
                    [np.sin(yaw)*deltak, 0],
                                [0, deltak]])
    return B

def ekf(z_k_observation_vector, state_estimate_k_minus_1, control_vector_k_minus_1, P_k_minus_1, dk):
    """
    Extended Kalman Filter. Fuses noisy sensor measurement to
    create an optimal estimate of the state of the robotic system.

    INPUT
        :param z_k_observation_vector The observation from the Odometry
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].
        :param state_estimate_k_minus_1 The state estimate at time k-1
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].
        :param control_vector_k_minus_1 The control vector applied at time k-1
            3x1 NumPy Array [v,v,yaw rate] in the global reference frame
            in [meters per second,meters per second,radians per second].
        :param P_k_minus_1 The state covariance matrix estimate at time k-1
            3x3 NumPy Array
        :param dk Time interval in seconds

    OUTPUT
        :return state_estimate_k near-optimal state estimate at time k
            3x1 NumPy Array ---> [meters,meters,radians]
        :return P_k state covariance_estimate for time k
            3x3 NumPy Array
    """
    ######################### Predict #############################
    # Predict the state estimate at time k based on the state
    # estimate at time k-1 and the control input applied at time k-1.
    state_estimate_k = A_k_minus_1 @ (state_estimate_k_minus_1) + (getB(state_estimate_k_minus_1[2],dk)) @ (control_vector_k_minus_1) + (process_noise_v_k_minus_1)
    print(f'State Estimate Before EKF={state_estimate_k}\r\n')
             
    # Predict the state covariance estimate based on the previous
    # covariance and some noise
    P_k = A_k_minus_1 @ P_k_minus_1 @ A_k_minus_1.T + (Q_k)
         
    ################### Update (Correct) ##########################
    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted 
    # the sensor measurements would be for the current timestep k.
    measurement_residual_y_k = z_k_observation_vector - (
            (H_k @ state_estimate_k) + (
            sensor_noise_w_k))
 
    print(f'Observation={z_k_observation_vector}\r\n')
             
    # Calculate the measurement residual covariance
    S_k = H_k @ P_k @ H_k.T + R_k
         
    # Calculate the near-optimal Kalman gain
    # We use pseudoinverse since some of the matrices might be
    # non-square or singular.
    K_k = P_k @ H_k.T @ np.linalg.pinv(S_k)
         
    # Calculate an updated state estimate for time k
    state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
     
    # Update the state covariance estimate for time k
    P_k = P_k - (K_k @ H_k @ P_k)
     
    # Print the best (near-optimal) estimate of the current state of the robot
    print(f'State Estimate After EKF={state_estimate_k}\r\n')
 
    # Return the updated state and covariance estimates
    return state_estimate_k, P_k    

############################################################################################

#function auto
def auto():
    print("starting autonomous movement.\r\n")
    #Count for Number of WPs assigned
    count = 0
    #Timestep for IMU EKF
    k = 1
    #compass bearing will be the offset for the rover to calculate the relative angle
    compass_bearing_arr = []
    bearing_arr=[] 
    z_k = []
    num_row = csvlen()
    sensor.mode = adafruit_bno055.COMPASS_MODE
    time.sleep(5)

    #initialize compass bearing for offset
    for i in range(100):
        compass_bearing_arr.append(sensor.euler[0])
    compass_bearing_std = np.std(compass_bearing_arr)
    
    #if compass standard deviation above 0.5 degrees reinitialize offset
    if compass_bearing_std > 0.5:
        print("bearing not accurate, reinitializing\r\n")
        auto()
    else:
        print("compass calibrated\r\n")
        compass_bearing = np.average(compass_bearing_arr)
        sensor.mode = adafruit_bno055.NDOF_MODE
        print("count: {}, num row: {}\r\n".format(str(count), str(num_row)))
    print("initializing\r\n")
    
    try:
        #keeps running if did not reach final location
        while count != num_row:
            print("moving autonmously \r\n")
            #Get initial lat and lon from GPS Coord
            current_lat, current_lon = gps()
            #Get relative angle from IMU
            current_bearing = sensor.euler[0] + compass_bearing
            
            #creates a list of sensor observations at successive time steps
            #each list within z_k is an observation vector
            yaw = radians(sensor.euler[0])
            pitch = sensor.euler[1]
            roll = sensor.euler[2]
            bearing_arr = pitch, roll, yaw
            #bearing_arr = np.transpose(bearing_arr, axes=None)
            z_k.append(bearing_arr)

          # z_k = np.array([[4.721,0.143,0.006], # k=1
           #        [9.353,0.284,0.007], # k=2
            #       [14.773,0.422,0.009],# k=3
             #      [18.246,0.555,0.011], # k=4
              #     [22.609,0.715,0.012]])# k=5
            
            #print('Sensor Observation', z_k, '\r\n')

            dk = 1
            
            # The estimated state vector at time k-1 in the global reference frame.
            # [x_k_minus_1, y_k_minus_1, yaw_k_minus_1]
            # [meters, meters, radians]
            state_estimate_k_minus_1 = np.array([0.0,0.0,0.0])
            
            # The control input vector at time k-1 in the global reference frame.
            # [v, yaw_rate]
            # [meters/second, radians/second]
            # In the literature, this is commonly u.
            # Because there is no angular velocity and the robot begins at the 
            # origin with a 0 radians yaw angle, this robot is traveling along 
            # the positive x-axis in the global reference frame.
            control_vector_k_minus_1 = np.array([4.5,0.0])
     
            # State covariance matrix P_k_minus_1
            # This matrix has the same number of rows (and columns) as the 
            # number of states (i.e. 3x3 matrix). P is sometimes referred
            # to as Sigma in the literature. It represents an estimate of 
            # the accuracy of the state estimate at time k made using the
            # state transition matrix. We start off with guessed values.
            P_k_minus_1 = np.array([[0.1,  0,   0],
                                    [  0,0.1,   0],
                                    [  0,  0, 0.1]])

            # Run the Extended Kalman Filter and store the
            # near-optimal state and covariance estimates
            for k, z_k in enumerate(z_k, start = 1):
                print(f'Timestep k = {k}\r\n')
                optimal_state_estimate_k, covariance_estimate_k = ekf(
                                z_k, # Most recent sensor measurement
                                state_estimate_k_minus_1, # Our most recent estimate of the state
                                control_vector_k_minus_1, # Our most recent control input
                                P_k_minus_1, # Our most recent state covariance matrix
                                dk) # Time interval
            
            current_bearing = degrees(optimal_state_estimate[2]) + compass_bearing
            print(current_bearing)
            break

            # Get ready for the next timestep by updating the variable values
            state_estimate_k_minus_1 =  optimal_state_estimate_k
            P_k_minus_1 = covariance_estimate_k

            #get and calculate the bearing the rover should be heading
            desired_lat, desired_lon = gpsread(count)
            desired_bearing = navigate(current_lat, current_lon, desired_lat, desired_lon)
            
            #Check for Objects in the way
            camera()
            
            if current_bearing < 0:
                current_bearing = 360 + current_bearing
            if current_bearing > 360:
                current_bearing = current_bearing - 360
            
            #Due to the calculation for the desired bearing, we need to ensure that the values align with the current
            #bearing to allow for accurate results
            if desired_bearing < 0:
                desired_bearing = 360 + desired_bearing
            
            #calculate the shortest angle to turn left or right
            left_close = current_bearing - desired_bearing
            right_close = desired_bearing - current_bearing
            if left_close < 0:
                left_close = left_close + 360
            if right_close < 0:
                right_close = right_close + 360
            print("current bearing: {}, desired bearing: {}\r\n".format(str(current_bearing), str(desired_bearing)))
            #Need to ensure that the rover does not read past the last point of the data to ensure no indexing errors
            if count >= num_row:
                count = num_row
                break
            #only move forward if bearing is reached to allow the rover to reach the desired point.
            if abs(desired_bearing - current_bearing) <= 2:
                forward()
               # time.sleep(0.3)
                current_bearing = sensor.euler[0]
                #only increment to the next waypoint if waypoint is reached.
                if abs(desired_lat - current_lat) < 0.000001 and abs(desired_lon - current_lon) < 0.000001:
                    print("wp reached\r\n")
                    brake()
                    current_lat, current_lon = gps()
                    desired_lat, desired_lon = gpsread(count)
                    desired_bearing = navigate(current_lat, current_lon, desired_lat, desired_lon)
                    count += 1
            elif right_close <= left_close:
                right()
               # time.sleep(0.3)
                current_bearing = sensor.euler[0]
                #allow the robot to reach closer to the desired bearing before determining whether bearing was truly
                #reached to account for fluctiations
                if abs(desired_bearing - current_bearing) <= 2:
                    print("bearing reached \r\n")
                    brake()
                    current_lat, current_lon = gps()
                    desired_bearing = navigate(current_lat, current_lon, desired_lat, desired_lon)
            elif left_close <= right_close:
                left()
               # time.sleep(0.3)
                current_bearing = sensor.euler[0]
                if abs(desired_bearing - current_bearing) <= 2:
                    print("bearing reached\r\n")
                    brake()
                    current_lat, current_lon = gps()
                    desired_bearing = navigate(current_lat, current_lon, desired_lat, desired_lon)
            elif abs(desired_lat - current_lat) < 0.000001 and abs(desired_lon - current_lon) < 0.000001:
                brake()
                current_lat, current_lon = gps()
                desired_bearing = navigate(current_lat, current_lon, desired_lat, desired_lon)
    except KeyboardInterrupt:
        hard_brake()
        print("exiting\r\n")
    
#function Manual
#Move Snobot with Keyboard only
def Manual():
    print("starting manual movement\n")
    print("press wasd to control the movement\n")
    try:
        while True:
            key = stdscr.getch()
            #Forward
            if key == ord('w'):
                forward()
            #Move Left
            elif key == ord('a'):
                left()
            #Move Right
            elif key == ord('d'):
                right()
            #Reverse
            elif key == ord('s'):
                reverse()
            else:
                brake()
                curses.endwin()
    #Stop all motors you stop program
    except KeyboardInterrupt:
        print("end\n")
        hard_brake()
        curses.endwin() 

def forward(): 
    print("forward\r\n")
    pca.servo[LeftMotor].angle = 97         #forward above 80 reverse below 15
    pca.servo[RightMotor].angle = 50        #forward below 50 reverse above 50
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def left():
    print("left\r\n")
    pca.servo[LeftMotor].angle = 50
    pca.servo[RightMotor].angle = 50
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def right():
    print("right\r\n")
    pca.servo[LeftMotor].angle = 95
    pca.servo[RightMotor].angle = 95
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def reverse():
    print("reverse\r\n")
    pca.servo[LeftMotor].angle = 50
    pca.servo[RightMotor].angle = 97
    pca.servo[Blower].angle = Max_Speed
    time.sleep(0.05)

def brake():
    print("brake\r\n")
    pca.servo[LeftMotor].angle = Brake
    pca.servo[RightMotor].angle = Brake
    pca.servo[Blower].angle = 120
    time.sleep(0.05)

def hard_brake():
    print("STOP\r\n")
    pca.servo[LeftMotor].angle = Brake
    pca.servo[RightMotor].angle = Brake
    pca.servo[Blower].angle = Brake
if __name__ == '__main__':
    init()
    main()

