#! /usr/bin/env python2
import sys
import tf
import os
# import cv2
import rospy
import rosbag
# import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
import time
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
# from cv_bridge import CvBridge
import numpy as np
import math
import argparse
from tqdm import tqdm, trange

velodyne_files = '/data/KITTI/data_odometry_velodyne/dataset/sequences'
calibration_files = '/data/KITTI/data_odometry_calib/dataset/sequences'
label_files = '/data/KITTI/data_odometry_labels/dataset/sequences'
prediction_files = '/data/KITTI/semantic_prediction/sequences'
bag_files = '/data/KITTI/kittiSemanticBag/prediction'

def parse_calibration(filename):
    """ read calibration file with given filename
        Returns
        -------
        dict
            Calibration matrices as 4x4 numpy arrays.
    """
    calib = {}

    calib_file = open(filename)
    for line in calib_file:
        key, content = line.strip().split(":")
        values = [float(v) for v in content.strip().split()]

        pose = np.zeros((4, 4))
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        calib[key] = pose

    calib_file.close()

    return calib

def parse_poses(filename, calibration):
    """ read poses file with per-scan poses from given filename
        Returns
        -------
        list
            list of poses as 4x4 numpy arrays.
    """
    file = open(filename)
    # Original odom of each cam frame relative to first cam frame
    original_poses = []
    # Transformed odom of each velodyne frame relative to first velodyne frames(set as world frame)
    world_poses = []

    Tr = calibration["Tr"]
    Tr_inv = np.linalg.inv(Tr)

    for line in file:
        values = [float(v) for v in line.strip().split()]

        pose = np.zeros((4, 4))
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        # Get each step velodyne pose relate to first velodyne frame(world)
        original_poses.append(pose)
        world_poses.append(np.matmul(Tr_inv, np.matmul(pose, Tr)))

    return original_poses, world_poses

def velodyne_to_bag(bag, seq, seq_frames, seq_times, velodyne_topic):
    
    iterable_scan = zip(seq_times, seq_frames)
    # bar = progressbar.ProgressBar()
    
    seq_progress = tqdm(len(seq_frames))
    for dt, filename in iterable_scan:
        if dt is None:
            continue
        
        # read labels
        # labelPath = os.path.join(label_files, seq, 'labels', filename[:-4]+'.label')
        labelPath = os.path.join(prediction_files, seq, 'predictions', filename[:-4]+'.label')
        frame_label = np.fromfile(labelPath, dtype=np.int32)
        sem_label = frame_label & 0xFFFF  # semantic label in lower half
        instance_id = frame_label & 0xFFFF0000
        
        # read binary data
        veloPath = os.path.join(velodyne_files, seq, 'velodyne', filename)
        raw_scan = np.fromfile(veloPath, dtype=np.float32)
        # original store x,y,z,intensity, each attribute for single point
        scan = raw_scan.reshape(-1, 4)
        
        pt_num = scan.shape[0]
        # print(len(raw_scan))
        # print(np.shape(raw_scan))
        # print("Total pt: {}".format(pt_num))
        # print("Scan shape: {}".format(np.shape(scan)))
        
        # expand each point field by label value and instance id
        label_scan = []
        for i in range(len(sem_label)):
            #Each pts attribute [x,y,z,intensity,label,id]
            pt_attribute = []
            for j in range(4):
                pt_attribute.append(scan[i][j])
            pt_attribute.append(sem_label[i])
            pt_attribute.append(instance_id[i])
            label_scan.append(pt_attribute)

        label_scan = np.array(label_scan)
        # print("Label scan shape: {}".format(np.shape(label_scan)))
        # for i in range(10):
        #     for j in range(label_scan.shape[1]):
        #         print(label_scan[i][j],)
        #     print("\n")

        #     for j in range(scan.shape[1]):
        #         print(scan[i][j],)
        #     print(sem_label[i])

        # exit(-1)
        # create header
        header = Header()
        header.frame_id = 'velodyne'
        header.stamp = rospy.Time.from_sec(dt)

        # fill sensor_msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1),
                PointField('label', 16, PointField.UINT32, 1),
                PointField('id', 20, PointField.UINT32, 1)]

        pcl_msg = pcl2.create_cloud(header, fields, label_scan)

        bag.write(velodyne_topic, pcl_msg, t=pcl_msg.header.stamp)

        seq_progress.update(1)

    return

def velodyne_pose_tf(bag, seq, seq_times, velodyne_topic):
    
    world_poses = get_pose_from_cam(seq)
    # print(len(world_poses),)
    # print(len(seq_times))
    iterable_scan = zip(seq_times, world_poses)

    for timestamp, poses in iterable_scan:
        tf_msg = TFMessage()
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
        tf_stamped.header.frame_id = '/world'
        tf_stamped.child_frame_id = '/velodyne'
        
        t = poses[0:3, 3]
        q = tf.transformations.quaternion_from_matrix(poses)
        transform = Transform()

        transform.translation.x = t[0]
        transform.translation.y = t[1]
        transform.translation.z = t[2]

        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]

        tf_stamped.transform = transform
        tf_msg.transforms.append(tf_stamped)

        bag.write('/tf', tf_msg, t=tf_msg.transforms[0].header.stamp)


def get_pose_from_cam(seq):
    # Read velodyne_to_cam transformation
    calib = {}
    calib_file = os.path.join(calibration_files, seq, 'calib.txt')
    calib = parse_calibration(calib_file)

    # Read poses at left cam frame
    poses = []
    poses_file = os.path.join(label_files, seq, 'poses.txt')
    # Convert pose relative to first velodyne frame coordinate(world) from cam to velodyne
    cam_poses, velo_poses = parse_poses(poses_file, calib)

    print(velo_poses[0],)
    print(velo_poses[-1])
    print(cam_poses[0],)
    print(cam_poses[-1])
    velo_dist = np.power((velo_poses[-1][:,3] - velo_poses[0][:,3]), 2)
    cam_dist = np.power((cam_poses[-1][:,3] - cam_poses[0][:,3]), 2)
    print("Traverse {} m in world frame".format(math.sqrt(np.sum(velo_dist))))
    print("Traverse {} m in cam   frame".format(math.sqrt(np.sum(cam_dist))))
    
    return velo_poses


if __name__ == "__main__":

    rospy.init_node('convertToKITTItoBag', anonymous=True)

    # bag = rosbag.Bag("kitti_{}_drive_{}_{}.bag".format(args.date, args.drive, args.kitti_type[4:]), 'w', compression=compression)
    # bag = rosbag.Bag("/data/KITTI/kittiSemanticBag/test_0_all.bag", 'w')

    sequences = sorted(os.listdir(prediction_files)) 
    # Using current time to set base_time of first frame in each sequence
    base_times = []

    print("Start read files!")
    print("Have {} sequence in total".format(len(sequences)))

    seq_progress = tqdm(len(sequences))
    for num, seq in enumerate(sequences):

        bag_name = os.path.join(bag_files, "kitti_semantic_seq_{}_prediction.bag".format(seq))
        bag = rosbag.Bag(bag_name, 'w')
        print("Recording bag: kitti_semantic_seq_{}_prediction.bag".format(seq))
        
        #  Produce each seq start time
        base_times.append(time.time())
        velo_frames = sorted(os.listdir(os.path.join(velodyne_files, seq, 'velodyne'))) 

        # Create timestamp of each pointcloud
        velo_datetimes = []
        with open(os.path.join(calibration_files, seq, 'times.txt')) as calib_file:
            lines = calib_file.readlines()
            
            for line in lines:
                #Empty line, only newline
                if len(line) == 1:
                    continue

                dt = base_times[num] + float(line)
                velo_datetimes.append(dt)
            # print("-")
            # print(base_times[num])
            # print(velo_datetimes[0])
            # print(velo_datetimes[-1])
            # print(velo_datetimes[-1]-velo_datetimes[0])
            # print("{}".format(float(lines[-1])-float(lines[0]), '.6f'))

        # Transform pointcloud and label to bag
        velodyne_to_bag(bag, seq, velo_frames, velo_datetimes, '/kitti/velodyne_points')

        # Transform visual odom to lidar odom tf
        velodyne_pose_tf(bag, seq, velo_datetimes, '/tf')

        # print("{}th seq, name: {}".format(num, seq))
        # print("Total have {} frames in seq {}".format(len(velo_frames), seq))
        # print("#"*40)

        print("## OVERVIEW ##")
        print(bag)
        bag.close()

        seq_progress.update(1)





