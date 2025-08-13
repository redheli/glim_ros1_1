#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>

// #include "livox_def_common.h"
#include <livox_ros_driver2/CustomMsg.h>
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

ros::Publisher pub;

void hap_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    ROS_INFO("hap handler");

    sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2());

    int plsize = msg->point_num;
    ROS_INFO("hap_handler: point num %d",plsize);
    pcl::PointCloud<PointXYZIRT> pcl_in;
    for(uint i=1; i<plsize; i++)
    {
        PointXYZIRT point;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        point.z = msg->points[i].z;
        point.intensity = msg->points[i].reflectivity;
        point.ring = (uint16_t)msg->points[i].line;
        point.time = msg->points[i].offset_time * float(1e-9); // ns to s
        pcl_in.push_back(point);
    }// end for plsize
   
   
    pcl::toROSMsg(pcl_in, *cloud);
    // set timestamp
    unsigned long timebase_ns = msg->timebase;
    cloud->header.stamp.fromNSec(timebase_ns);
    cloud->header.frame_id = msg->header.frame_id;
    pub.publish(cloud);
}



int main(int argc, char** argv)
{
    std::string custom_msg_topic = "/livox/lidar";
    if (argc > 1){
        custom_msg_topic = std::string(argv[1]);
    }
    ROS_INFO("Livox Custom Message Topic <%s>",custom_msg_topic.c_str());

    ros::init(argc, argv, "sub_pc");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(custom_msg_topic, 1000, hap_handler);

    pub = n.advertise<sensor_msgs::PointCloud2> ("livox/pc2", 1);

    ros::spin();

    return 0;
}