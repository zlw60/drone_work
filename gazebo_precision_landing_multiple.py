#!/usr/bin/env python

# Get all the inports that are needed
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import  mavutil
from array import array


# Initialize  dronekit vehicle

#SITL Gazebo
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Real drone
#connection_string = "/dev/ttyACM0"
#baud_rate = 115200
#vehicle = connect(connection_string,baud=baud_rate, wait_ready=True)


vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 30 #cm/s

takeoff_height = 10 #m
velocity = 0.5 #m/s


# Create a few variables that will be needed.
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=300)

ids_to_find = [392,288,264,226,201,162,138,129]
marker_sizes = [120,100,80,60,40,20,10,8] #cm
marker_heights = [52,43,35,26,18,8,3,2] #m

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
verticle_res = 480

horizontal_fov = 62.2 * (math.pi / 180)
verticle_fov = 48.8 * (math.pi /180)

found_count = 0
notfound_count = 0

# Get camera parameters

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]

np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)


time_last = 0
time_to_wait = .1  #100 ms
# Functions

#def get_distance_meters(targetLocation,currentLocation):
#    dLat = targetLocation.lat - currentLocation.lat
#    dLon = targetLocation.lon - currentLocation.lon

#    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

#def goto(targetLocation):
#    distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
#    vehicle.simple_goto(targetLocation)

#    while vehicle.mode.name == 'GUIDED':
#        currentDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
#        if currentDistance < distanceToTargetLocation *.02:
#            print('Reached Target Location')
#            time.sleep(1)
#            break
#        time.sleep(1)
#        break
#    return None


def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print ('Waiting for vehicle to become armable')
        time.sleep(1)
    print ('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode != 'GUIDED':
        print ('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print ('Vehicle now in GUIDED mode.')

    vehicle.armed = True
    while vehicle.armed == False:
        print ('Waiting for vehicle to become armed.')
        time.sleep(1)
    print ('Look out! Props are spinning.')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print ('Current Altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95*targetHeight:
            break
        time.sleep(1)
    print ('Target altitude reached!')

    return None



#def send_local_ned_velocity(vx,vy,vz):
#    msg = vehicle.message_factory.set_position_target_local_ned_encode(
#    0,
#
#    0,
#    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
#    0b0000111111000111,
#    0,
#    0,
#    0,
#    vx,
#    vy,
#    vz,
#    0,0,0,0,0)
#    vehicle.send_mavlink(msg)
#    vehicle.flush()



def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
    0,
    0,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    x,
    y,
    0,0,0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict, parameters=parameters)

        altitude = vehicle.location.global_relative_frame.alt # meters

        id_to_find = 0
        marker_size = 0
        maker_height = 0

        # altitude is more than 43 meters
        if altitude > marker_heights[1]:
            id_to_find = ids_to_find[0]
            marker_size = marker_sizes[0]
            marker_height = marker_sizes[0]

        # altitude is more than 35 meters but less than 43 meters
        elif altitude > marker_heights[2]:
            id_to_find = ids_to_find[1]
            marker_size = marker_sizes[1]
            marker_height = marker_sizes[1]

        # altitude is more than 26 meters but less than 35 meters
        elif altitude > marker_heights[3]:
            id_to_find = ids_to_find[2]
            marker_size = marker_sizes[2]
            marker_height = marker_sizes[2]

        # altitude is more than 18 meters but less than 26 meters
        elif altitude > marker_heights[4]:
            id_to_find = ids_to_find[3]
            marker_size = marker_sizes[3]
            marker_height = marker_sizes[3]

        # altitude is more than 8 meters but less than 18 meters
        elif altitude > marker_heights[5]:
            id_to_find = ids_to_find[4]
            marker_size = marker_sizes[4]
            marker_height = marker_sizes[4]

        # altitude is more than 3 meters but less than 8 meters
        elif altitude > marker_heights[6]:
            id_to_find = ids_to_find[5]
            marker_size = marker_sizes[5]
            marker_height = marker_sizes[5]

        # altitude is more than 2 meters but less than 3 meters
        elif altitude > marker_heights[7]:
            id_to_find = ids_to_find[6]
            marker_size = marker_sizes[6]
            marker_height = marker_sizes[6]

        # altitude less than 2 meters
        elif altitude < marker_heights[7]:
            id_to_find = ids_to_find[7]
            marker_size = marker_sizes[7]
            marker_height = marker_sizes[7]

        ids_array_index = 0
        found_id = 0
        print ('Looking for marker: ' +str(id_to_find))

        try:
            if ids is not None:
                for id in ids:
                    if id == id_to_find:
                        corners_single = [corners[ids_array_index]]
                        corners_single_np = np.asarray(corners_single)

                        ret = aruco.estimatePoseSingleMarkers(corners_single,marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                        x = '{:.2f}'.format(tvec[0])
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])

                        xsum = 0
                        ysum = 0

                        xsum = corners_single_np[0][0][0][0] + corners_single_np[0][0][1][0] + corners_single_np[0][0][2][0] + corners_single_np[0][0][3][0]
                        ysum = corners_single_np[0][0][0][1] + corners_single_np[0][0][1][1] + corners_single_np[0][0][2][1] + corners_single_np[0][0][3][1]

                        xavg = xsum /4
                        yavg = ysum /4

                        xang = (xavg - horizontal_res*0.5)* horizontal_fov/horizontal_res
                        yang = (yavg - verticle_res*0.5)* verticle_fov/verticle_res

                        if vehicle.mode != 'LAND':
                            vehicle.mode = VehicleMode('LAND')
                            while vehicle.mode != 'LAND':
                                time.sleep(1)
                            print ('Vehicle in LAND mode')
                            send_land_message(xang, yang)
                        else:
                            send_land_message(xang, yang)

                        marker_posistion = 'MARKER POSISTION: x='+x+' y='+y+' z='+z

                        aruco.drawDetectedMarkers(np_data, corners)
                        aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
                        cv2.putText(np_data, marker_posistion, (10,50),0,.7, (255,0,0), thickness = 2)
                        print(marker_posistion)
                        print ('FOUND COUNT: ' +str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))

                        found_count = found_count+1
                        found_id = 1
                        break
                    ids_array_index = ids_array_index +1
                if found_id==0:
                    notfound_count = notfound_count+1
            else:
                notfound_count = notfound_count+1
        except Exception as e:
            print('Target likely not found')
            print(e)
            notfound_count = notfound_count+1
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None


def landing():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()


if __name__ == '__main__':

    try:

        arm_and_takeoff(takeoff_height)
        time.sleep(10)
        landing()

    except rospy.ROSInterruptException:
        pass
