//  It's for transforming point cloud label to corresponding color coded map and republish in rgb pointcloud

#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "color_table.h"

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
string INPUT_TOPIC, OUTPUT_TOPIC, OUTPUT_LABEL_TEXT_TOPIC;
ros::Subscriber sub_cloud;
ros::Publisher pub_cloud, pub_label;
static colorTable table;

void publishLabelText(sensor_msgs::PointCloud2 cloud_msg)
{
    visualization_msgs::MarkerArray textArray;
    sensor_msgs::PointCloud2Iterator<float> iter_ori_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_ori_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_ori_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_ori_label(cloud_msg, "label");
    int idx = 0;
    for(; iter_ori_label != iter_ori_label.end(); ++iter_ori_label, ++iter_ori_x, ++iter_ori_y, ++iter_ori_z)
    {
        string label_name = table.getLabels(*iter_ori_label);
        visualization_msgs::Marker textMarker;
        textMarker.header = cloud_msg.header;
        textMarker.ns = "label";
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.lifetime = ros::Duration(0.1);
        textMarker.scale.z = 1.0;
        textMarker.color.r = 0.5;
        textMarker.color.g = 0.5; 
        textMarker.color.b = 0.0;
        textMarker.color.a = 1.0; 
        textMarker.id = idx++;

        textMarker.pose.position.x = *iter_ori_x;
        textMarker.pose.position.y = *iter_ori_y;
        textMarker.pose.position.z = *iter_ori_z + 1.0;

        textMarker.text = label_name;

        textArray.markers.push_back(textMarker);
    }
    pub_label.publish(textArray);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cout<<"Get lidar at: "<< msg->header.stamp << endl;
    sensor_msgs::PointCloud2 cloud_msg = *msg;
    sensor_msgs::PointCloud2Iterator<float> iter_ori_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_ori_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_ori_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_ori_label(cloud_msg, "label");

    int pt_num = cloud_msg.width * cloud_msg.height;
    
    // Create pointcloud
    sensor_msgs::PointCloud2 out_cloud;
    out_cloud.width = cloud_msg.width;
    out_cloud.height = cloud_msg.height;
    out_cloud.header = cloud_msg.header;
    out_cloud.is_dense = false;
    // Create field manually
    sensor_msgs::PointCloud2Modifier modifier(out_cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "rgb", 1, sensor_msgs::PointField::FLOAT32);
    // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
    // You have to be aware that the following function does add extra padding for backward compatibility though
    // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
    // 2 is for the number of fields to add
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
    // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
    // and RGBA as A,R,G,B)
    sensor_msgs::PointCloud2Iterator<float> iter_x(out_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(out_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(out_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(out_cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(out_cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(out_cloud, "b");

    for(; iter_ori_label != iter_ori_label.end(); ++iter_ori_label, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b,
                                                ++iter_ori_x, ++iter_ori_y, ++iter_ori_z)
    {
        int id = *iter_ori_label;
        std::vector<int> color = table.getColor(id);
        if (color[0] == -1)
            continue;
        *iter_x = *iter_ori_x;
        *iter_y = *iter_ori_y;
        *iter_z = *iter_ori_z;
        *iter_r = static_cast<uint8_t>(color[2]);
        *iter_g = static_cast<uint8_t>(color[1]);
        *iter_b = static_cast<uint8_t>(color[0]);
    }
    pub_cloud.publish(out_cloud);
    // publishLabelText(cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_kitti_bag");
    ros::NodeHandle nh("~");
    nh.param<string>("input_topic", INPUT_TOPIC, "/kitti/velodyne_points");
    nh.param<string>("output_topic", OUTPUT_TOPIC, "/colored" + INPUT_TOPIC);
    nh.param<string>("output_label_text_topic", OUTPUT_LABEL_TEXT_TOPIC, "/label_text" + INPUT_TOPIC);

    sub_cloud = nh.subscribe(INPUT_TOPIC, 1, &cloudCallback);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(OUTPUT_TOPIC, 1);
    pub_label = nh.advertise<visualization_msgs::MarkerArray>(OUTPUT_LABEL_TEXT_TOPIC, 1);


    ros::Rate loop(10);
    while (ros::ok())
    {
        ros::spin();
        loop.sleep();
    }
    return 0;
}