#! /usr/bin/env python3
import math
import sys
import rospy
import serial
from sensor_msgs.msg import Imu


rospy.init_node('sparkfun_imu_driver')
device = rospy.get_param('serial_device', default='/dev/ttyUSB0')
baud_rate = rospy.get_param('baud_rate', default=500000)
pub_imu = rospy.Publisher('imu', Imu, queue_size=1)

# Connect to the IMU
while not rospy.is_shutdown():
    try:
        serial_port = serial.Serial(port=device, baudrate=baud_rate, timeout=1)
        rospy.loginfo('Successfully connected to IMU!')
        break
    except serial.serialutil.SerialException as e:
        rospy.logwarn(f'Failed to connect to IMU: {e}')
        rospy.sleep(1)

# Read and parse ASCII data on the fly
rospy.loginfo('Starting to read data from IMU')
while not rospy.is_shutdown():
    if not serial_port.is_open:
        # Device connection lost... shutdown node
        rospy.signal_shutdown('Lost connection to IMU')
        sys.exit(1)

    try:
        line = serial_port.readline().decode()
    except UnicodeDecodeError as e:
        rospy.logwarn(e)
        continue

    ascii_fields = [s.strip() for s in line.split(',')]
    if len(ascii_fields) != 14:
        continue

    accel_scale_factor = 0.01
    gyro_scale_factor = math.pi / 180 # dps --> rad/s

    try:
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time().now()
        imu_msg.header.frame_id = 'map'
        
        # Negate x and z components to rotate sensor's reference frame by 180 degrees about y
        imu_msg.linear_acceleration.x = -accel_scale_factor * float(ascii_fields[2])
        imu_msg.linear_acceleration.y = accel_scale_factor * float(ascii_fields[3])
        imu_msg.linear_acceleration.z = -accel_scale_factor * float(ascii_fields[4])
        imu_msg.angular_velocity.x = -gyro_scale_factor * float(ascii_fields[5])
        imu_msg.angular_velocity.y = gyro_scale_factor * float(ascii_fields[6])
        imu_msg.angular_velocity.z = -gyro_scale_factor * float(ascii_fields[7])
        pub_imu.publish(imu_msg)
    except ValueError as e:
        pass