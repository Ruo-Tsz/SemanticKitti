'''
It's for extract pointcloud from bag and store in .bin file
Also include test reading and pub stored .bin cloud
'''


#! /usr/bin/env python2
import rospy
import rosbag
import os
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import pypcd
import numpy as np
import argparse

# Extract_flag = False


bag_file = '/data/bags/itri/moto'
input_bag = '2020-09-11-17-31-33.bag'
# output_bin_dir
output_file = '/data/itri/motos/31_bag_s260-310'
# total num wanna extrack
frame_num = 500
# from bag start at (start_time) sec
start_time = 260

cloud_pub = rospy.Publisher("velodyne_points", PointCloud2, queue_size = 1000)

def readFrames():
    frames_list = sorted(os.listdir(output_file))
    # print(frames_list[0][:-4])

    scans_list = []
    for f in frames_list:
        scanPath = os.path.join(output_file, f)
        raw_scan = np.fromfile(scanPath, dtype=np.float32)
        # (N, 4)
        scan = raw_scan.reshape(-1, 4)
        scans_list.append(scan)

    return frames_list, scans_list

def extract_cloud(frame_num, start_time = 0):
    # extract pc from bag
    bag_start_time = 0
    get_bag_time = False
    count = 0
    for topic, msg, t in rosbag.Bag(os.path.join(bag_file, input_bag)).read_messages():
        if get_bag_time == False:
            try:
                bag_start_time = msg.header.stamp
                get_bag_time = True
                print('Get bag_start_time: ', bag_start_time.to_nsec())
            except Exception as e:
                print(e)
                continue
        
        try:
            if topic == '/velodyne_points':
                stamp = msg.header.stamp
                if abs(stamp.to_sec() - bag_start_time.to_sec()) < start_time:
                    continue 

                output_bin = str(stamp.to_nsec())
                # print( type(stamp.to_nsec()))

                pc = pypcd.PointCloud.from_msg(msg)
                # (N,) array
                x = pc.pc_data['x']
                y = pc.pc_data['y']
                z = pc.pc_data['z']
                intensity = pc.pc_data['intensity']
                # (N*4,) array
                arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
                # array need to in order [x1, y1, z1, i1, x2, y2, z2, i2...]
                # fill arr[0, 4, 8...]
                arr[::4] = x
                # fill arr[1, 5, 9...] 
                arr[1::4] = y
                arr[2::4] = z
                # kitti format intensity [0, 1], not [0, 255]
                arr[3::4] = intensity / 255 
                # print(arr.shape)
                # print(intensity.shape)
                # save it in a binary file using a float32 format
                arr.astype('float32').tofile(os.path.join(output_file, output_bin+'.bin'))
                count = count + 1

                if count == frame_num:
                    break
                # print(intensity[0])
                # print(arr[3])       

        except Exception as e:
            print(e)

if __name__ == "__main__":

    rospy.init_node('pub_bin_node', anonymous=True)

    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-e",
                    "--extract",
                    action="store_true",
                    help="Type -e for extracting")
    group.add_argument("-l",
                    "--load",
                    action="store_true",
                    help="Type -l reading")
    # parser.add_argument('extract_flag', help='True for extracting, False for reading', type=bool)
    args = parser.parse_args()
    print('extract_flag: ', args.extract)
    print('load_flag: ', args.load)

    # extract pc from bag
    if args.extract:
        print("Extract cloud From bag: ",os.path.join(bag_file, input_bag))
        extract_cloud(frame_num, start_time)
    # read stored bin
    else:
        print("Loading cloud From: ",output_file)
        frames_list, scans_list = readFrames()
        assert len(scans_list) == len(frames_list)

        # # 10hz
        pub_count = 0
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            if pub_count == len(frames_list):
                pub_count = 0
            
            header = Header()
            header.frame_id = 'velodyne'
            header.stamp.secs = int(int(frames_list[pub_count][:-4]) / 1e9)
            header.stamp.nsecs= int(int(frames_list[pub_count][:-4]) % 1e9)

            print(header.stamp.secs)
            print(header.stamp.nsecs)

            # fill sensor_msg
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]

            pcl_msg = pcl2.create_cloud(header, fields, scans_list[pub_count])

            # time is a little deviated in nsecs of last 3 digits when reading in
            print(pcl_msg.header)
            print(pcl_msg.width * pcl_msg.height, ' points')
            cloud_pub.publish(pcl_msg)
            rate.sleep()
            pub_count = pub_count + 1


