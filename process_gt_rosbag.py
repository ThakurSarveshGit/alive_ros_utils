#!/usr/bin/env python

import rosbag.bag
import rospy
import rosbag
import csv
import sys
import os
import pyproj
import argparse
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

# Initialize UTM projection
proj = pyproj.Proj(proj='utm', zone=33, ellps='WGS84')

# Global variables for tracking IMU-based position
last_time = None
position = np.zeros(3)
velocity = np.zeros(3)
orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion [x, y, z, w]

def gps_callback(msg, gps_file):
    timestamp = msg.header.stamp.to_sec()
    x, y = proj(msg.longitude, msg.latitude)
    tum_data = [timestamp, x, y, 0.0, 0.0, 0.0, 0.0, 1.0]
    
    with open(gps_file, 'a') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow(tum_data)


from scipy.spatial.transform import Rotation as R
import numpy as np

# Initialize global variables for position and orientation
current_position = np.array([0.0, 0.0, 0.0])
current_orientation = R.from_quat([0.0, 0.0, 0.0, 1.0])
last_timestamp = None

# IMU callback does not work correctly
def imu_callback(msg, imu_file):
    global current_position, current_orientation, last_timestamp
    
    timestamp = msg.header.stamp.to_sec()

    if last_timestamp is None:
        last_timestamp = timestamp
        return
    
    # Time difference
    dt = timestamp - last_timestamp
    last_timestamp = timestamp

    # Extract orientation as a quaternion
    imu_orientation = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    # Update orientation
    current_orientation = imu_orientation

    # Rotate the velocity (angular_velocity) and integrate to update position
    linear_velocity = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    velocity_in_world_frame = current_orientation.apply(linear_velocity) * dt
    current_position += velocity_in_world_frame

    # TUM format: timestamp x y z qx qy qz qw
    tum_data = [timestamp] + list(current_position) + list(current_orientation.as_quat())
    
    # Write to TUM file
    with open(imu_file, 'a') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow(tum_data)

def check_and_handle_existing_files(gps_file, imu_file):
    if os.path.exists(gps_file) or os.path.exists(imu_file):
        print("Error: One or both of the trajectory files already exist.")
        sys.exit(1)

def process_bag(args):
    rospy.init_node('process_gt_rosbag', anonymous=True)

    check_and_handle_existing_files(args.gps_file, args.imu_file)

    with rosbag.Bag(args.bag_file, 'r') as bag:
        print(f"Processing bag file[{args.bag_file}] for ground truth")
        total_msgs = bag.get_message_count()
        with tqdm(total=total_msgs, desc="Processing Bag", unit="msg") as pbar:
            for topic, msg, t in bag.read_messages():
                if topic == args.gps_topic:
                    gps_callback(msg, args.gps_file)
                elif topic == args.imu_topic:
                    imu_callback(msg, args.imu_file)
                pbar.update(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Process a ROS bag file to extract GPS and IMU data.")
    parser.add_argument('--bag_file', default="eight_shaped_loop.bag", help="Path to the ROS bag file")
    parser.add_argument('--gps_file', default='gps_trajectory.tum', help="Output file for GPS trajectory")
    parser.add_argument('--imu_file', default='imu_trajectory.tum', help="Output file for IMU trajectory")
    parser.add_argument('--imu_topic', default='/mavros/imu/data', help="IMU topic to read from")
    parser.add_argument('--gps_topic', default='/mavros/global_position/global', help="GPS topic to read from")
    
    args = parser.parse_args()

    process_bag(args)



# to call the script:
# python3 process_gt_rosbag.py --bag_file $win/downloads/eight_shaped_loop.bag