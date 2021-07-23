#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
 
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "color_table.h"

using namespace std;
ros::Subscriber sub_cloud_;
ros::Publisher pub_cloud_;
string INPUT_TOPIC, OUTPUT_TOPIC;
static colorTable table;
static vector<string> filtered_label = {
        "unlabeled",
        "outlier",
        "road",
        "parking",
        "sidewalk",
        "other-ground",
        "building",
        "fence",
        "other-structure",
        "lane-marking",
        "vegetation",
        "trunk",
        "terrain",
        "pole",
        "traffic-sign"
};

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_cloud)
{
    std::cout << "Get input: " << in_cloud->header.stamp << std::endl;
    sensor_msgs::PointCloud2 cloud_msg = *in_cloud;
    sensor_msgs::PointCloud2Iterator<float> in_iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> in_iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> in_iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> in_iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> in_iter_label(cloud_msg, "label");
    sensor_msgs::PointCloud2Iterator<uint32_t> in_iter_id(cloud_msg, "id");

    // cteate output cloud
    sensor_msgs::PointCloud2 out_cloud;
    out_cloud.header = cloud_msg.header;
    out_cloud.width = cloud_msg.width;
    out_cloud.height = cloud_msg.height;
    out_cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(out_cloud);
    modifier.setPointCloud2Fields(6, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "intensity", 1, sensor_msgs::PointField::FLOAT32,
                                    "label", 1, sensor_msgs::PointField::UINT32,
                                    "id", 1, sensor_msgs::PointField::UINT32);

    sensor_msgs::PointCloud2Iterator<float> out_iter_x(out_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_iter_y(out_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_iter_z(out_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> out_iter_intensity(out_cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> out_iter_label(out_cloud, "label");
    sensor_msgs::PointCloud2Iterator<uint32_t> out_iter_id(out_cloud, "id");


    // sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg, temp_in_cloud);
    int pt_num_ = cloud_msg.width * cloud_msg.height;

    int count = 0;
    for(; in_iter_label != in_iter_label.end(); 
        ++in_iter_x, ++in_iter_y, ++in_iter_z, ++in_iter_intensity, ++in_iter_label, ++in_iter_id)
    {
        string label = table.getLabels(*in_iter_label);
        std::vector<string>::iterator it = std::find(filtered_label.begin(), filtered_label.end(), label);
        if (it == filtered_label.end())
        {
            *out_iter_x = *in_iter_x;
            *out_iter_y = *in_iter_y;
            *out_iter_z = *in_iter_z;
            *out_iter_intensity = *in_iter_intensity;
            *out_iter_label = *in_iter_label;
            *out_iter_id = *in_iter_id;

            ++out_iter_x;
            ++out_iter_y;
            ++out_iter_z;
            ++out_iter_intensity;
            ++out_iter_label;
            ++out_iter_id;
            count++;
        }
    }
    std::cout << "At:           " << out_cloud.header.stamp << std::endl;
    std::cout << "Original have " << pt_num_ << " segmented pts" << std::endl;
    std::cout << "Finally  have " << count << " segmented pts" << std::endl;

    pub_cloud_.publish(out_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_cloud_filter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh_("~");
    
    nh.param<string>("input_topic", INPUT_TOPIC, "/kitti/velodyne_points");
    nh.param<string>("output_topic", OUTPUT_TOPIC, "/filtered" + INPUT_TOPIC);

    sub_cloud_ = nh.subscribe(INPUT_TOPIC, 1, &cloudCallback);
    pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(OUTPUT_TOPIC, 1);

    ros::Rate loop(10);
    while (ros::ok())
    {
        ros::spin();
        loop.sleep();
    }
    
    return 0;
}

