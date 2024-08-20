#!/usr/bin/env python3

"""
Extracts data from ALIVE car ROSBAG dataset in DROID compatible format.

This script is designed to extract data from the ALIVE CAR ROSBAG dataset and convert it into a format compatible with DROID SLAM.
It provides functionality to process the dataset and extract relevant information, including RGB images, depth images, LiDAR data, IMU data, and GPS data.

Usage:
    `python3 extract_data_droid.py $win/Downloads/IIITD-BagFiles/eight_loop.bag --extract_image --extract_depth --extract_gps --out_dir $win/Downloads/IIITD-BagFiles/eight_loop`

Author: Sarvesh Thakur
Date: 20-08-2024
"""

import os
import rosbag
import rospy
from tqdm import tqdm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import argparse
import pyproj
import csv
import numpy as np
import subprocess
import shutil

cv_bridge = CvBridge() # ros image to cv image converter
proj = pyproj.Proj(proj='utm', zone=33, ellps='WGS84') # Initialize UTM projection

# Function to extract and save images(rgb and depth) from the ROSBAG dataset
def extract_and_save_images(bag, topic, out_dir, encoding, file_prefix):
    
    out_topic_dir = os.path.join(out_dir, file_prefix)
    os.makedirs(out_topic_dir, exist_ok=True)
    file_list = open(os.path.join(out_dir, f'{file_prefix}.txt'), 'w')
    
    for _, msg, t in tqdm(bag.read_messages(topics=[topic]), desc=f"Extracting {file_prefix} images data from topic {topic}", total=bag.get_message_count(topic)):
        fn = format(((rospy.rostime.Time.to_nsec(t)/1e9) - 1580300000), '.6f')
        try:
            cvimg = cv_bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            out_fn = os.path.join(out_topic_dir, fn + '.png')
            file_list.write(fn + f' {file_prefix}/' + fn + '.png' + '\n')
            cv2.imwrite(out_fn, cvimg)        
        except Exception as e:
            print(e)
    
    file_list.close()


def extract_and_save_gps(bag, topic, out_dir, format='tum'):
    
    os.makedirs(out_dir, exist_ok=True)
    gps_file = os.path.join(out_dir, 'gps.'+format)
    if not os.path.exists(gps_file):
        f = open(gps_file, 'w')
        f.close()
    else:
        raise FileExistsError(f"File {gps_file} already exists.")
    
    def gps_callback(msg, gps_file):
        timestamp = msg.header.stamp.to_sec()
        x, y = proj(msg.longitude, msg.latitude)
        tum_data = [timestamp, x, y, 0.0, 0.0, 0.0, 0.0, 1.0]
        
        with open(gps_file, 'a') as f:
            writer = csv.writer(f, delimiter=' ')
            writer.writerow(tum_data)
    
    for _, msg, _ in tqdm(bag.read_messages(topics=[topic]), desc=f"Extracting GPS data from topic {topic}", total=bag.get_message_count(topic)):
        gps_callback(msg, gps_file)


# Function to run kiss_icp_pipeline
def run_kiss_icp_pipeline(args):
    lidar_topic, bag_file, visualize = args.lidar_topic, args.bag_file, args.visualize

    cmd = ['kiss_icp_pipeline', '--topic', lidar_topic, bag_file]
    if visualize:
        cmd.insert(3, '--visualize')
    subprocess.run(cmd)
    
    # Rename and move the 'results' folder
    if os.path.exists('results'):
        os.rename('results', 'kiss_icp_results')
        shutil.move('kiss_icp_results', args.out_dir)

    # # Run evo_traj command on the tum trajectory
    # bag_file_name = os.path.splitext(os.path.basename(args.bag_file))[0]
    # tum_file = os.path.join(args.out_dir, 'kiss_icp_results', '2024-08-20_23-45-40', f'{bag_file_name}_tum.txt')
    # evo_traj_cmd = f'evo_traj tum {tum_file}'
    # subprocess.run(evo_traj_cmd, shell=True)


# Function to extract data from the ROSBAG dataset
def extract_data(args):
    bag = rosbag.Bag(args.bag_file)

    # Extract image data if specified
    if args.extract_image:
        extract_and_save_images(bag, args.image_topic, args.out_dir, 'bgr8', 'rgb')
        
    # Extract depth data if specified
    if args.extract_depth:
        extract_and_save_images(bag, args.depth_topic, args.out_dir, 'passthrough', 'depth')
        
    # Extract LiDAR data if specified
    if args.extract_lidar:
        raise NotImplementedError("Functionality not implemented for extracting LiDAR data.")
        
    # Extract IMU data if specified
    if args.extract_imu:
        raise NotImplementedError("Functionality not implemented for extracting IMU data.")
        
    # Extract GPS data if specified
    if args.extract_gps:
        extract_and_save_gps(bag, args.gps_topic, args.out_dir, format='tum')

    # Run kiss_icp_pipeline if specified
    if args.run_kiss_icp:
        run_kiss_icp_pipeline(args)

    bag.close()

# Main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract data from ALIVE ROSBAG dataset in DROID compatible format.')
    parser.add_argument('bag_file', type=str, help='Path to the ROSBAG dataset file')
    parser.add_argument('--image_topic', type=str, default='/camera/color/image_raw', help='Name of the topic containing image data')
    parser.add_argument('--depth_topic', type=str, default='/camera/depth/image_rect_raw', help='Name of the topic containing depth data')
    parser.add_argument('--lidar_topic', type=str, default='/lidar102/velodyne_points', help='Name of the topic containing LiDAR data')
    parser.add_argument('--imu_topic', type=str, default='/mavros/imu/data', help='Name of the topic containing IMU data')
    parser.add_argument('--gps_topic', type=str, default='/mavros/global_position/global', help='Name of the topic containing GPS data')
    parser.add_argument('--out_dir', type=str, default='.', help='Output directory for saving extracted data')
    parser.add_argument('--extract_image', action='store_true', help='Extract image data')
    parser.add_argument('--extract_depth', action='store_true', help='Extract depth data')
    parser.add_argument('--extract_lidar', action='store_true', help='Extract LiDAR data')
    parser.add_argument('--extract_imu', action='store_true', help='Extract IMU data')
    parser.add_argument('--extract_gps', action='store_true', help='Extract GPS data')
    parser.add_argument('--run_kiss_icp', action='store_true', help='Run kiss_icp_pipeline to generate ground truth trajectory')
    parser.add_argument('--visualize', action='store_true', help='Visualize the trajectory being generated by kiss_icp_pipeline')
    args = parser.parse_args()

    extract_data(args)


# python3 extract_data_droid.py $win/Downloads/IIITD-BagFiles/eight_loop.bag --extract_image --extract_depth --out_dir $win/Downloads/IIITD-BagFiles/eight_loop