#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <string>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <cmath>

struct ijd
{
    int i, j;
    float d;
};

class LidarToImageNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher image_publisher;
    ros::Subscriber lidar_subscriber;
    std::string input_topic, output_topic;
    pcl::PointCloud<pcl::PointXYZ> pcl;
    sensor_msgs::ImagePtr image_msg;
    cv_bridge::CvImage image;
    int height, width;
    float max_vertical_angle, min_vertical_angle, vertical_angle_range;
    float max_horizontal_angle, min_horizontal_angle, horizontal_angle_range;
    float max_distance;
    float void_value;
    bool invert_distance, normalize_image;

public:
    LidarToImageNode(void)
    {
        nh = ros::NodeHandle("~");
        nh.param<std::string>("input_topic", input_topic, "lidar_reading");
        nh.param<std::string>("output_topic", output_topic, "depth_image");
        nh.param<int>("height", height, 16);
        nh.param<int>("width", width, 720);
        nh.param<float>("max_vertical_angle", max_vertical_angle, 15.0);
        nh.param<float>("min_vertical_angle", min_vertical_angle, -15.0);
        nh.param<float>("max_horizontal_angle", max_horizontal_angle, 360);
        nh.param<float>("min_horizontal_angle", min_horizontal_angle, 0);
        nh.param<float>("max_distance", max_distance, 50);
        nh.param<float>("void_value", void_value, 0.0);
        nh.param<bool>("invert_distance", invert_distance, true);
        nh.param<bool>("normalize_image", normalize_image, false);
        vertical_angle_range = max_vertical_angle - min_vertical_angle;
        horizontal_angle_range = max_horizontal_angle - min_horizontal_angle;
        ROS_DEBUG("Input topic:  %s", input_topic.data());
        ROS_DEBUG("Output topic: %s", output_topic.data());
        ROS_DEBUG("Height: %i", height);
        ROS_DEBUG("Width: %i", width);
        ROS_DEBUG("Min horizontal angle: %f", min_horizontal_angle);
        ROS_DEBUG("Max horizontal angle: %f", max_horizontal_angle);
        ROS_DEBUG("horizontal angle range: %f", horizontal_angle_range);
        ROS_DEBUG("Min vertical angle: %f", min_vertical_angle);
        ROS_DEBUG("Max vertical angle: %f", max_vertical_angle);
        ROS_DEBUG("Vertical angle range: %f", vertical_angle_range);
        lidar_subscriber = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, &LidarToImageNode::ptcl_callback, this);
        image_publisher = nh.advertise<sensor_msgs::Image>(output_topic, 1);
    }
    ijd point_to_index(pcl::PointXYZ pt)
    {
        return coordinates_to_index(pt.x, pt.y, pt.z);
    }
    ijd coordinates_to_index(float x, float y, float z)
    {
        float dist = std::sqrt(x * x + y * y + z * z);
        float dist_to_z_axis = std::sqrt(x * x + y * y);
        float theta_angle_deg = std::atan2(y, x) / M_PI * 180.0;
        theta_angle_deg = std::fmod(theta_angle_deg + 360.0, 360.0);
        float delta_angle_deg = std::atan2(z, dist_to_z_axis) / M_PI * 180;
        int delta_idx = std::floor((delta_angle_deg - min_vertical_angle) / (vertical_angle_range) * (height - 1));
        float reduced_theta = theta_angle_deg / horizontal_angle_range;
        int theta_idx = std::floor(reduced_theta * (width - 1));
        // theta_idx = (width - 1) - theta_idx;
        delta_idx = (height - 1) - delta_idx;
        ijd idx;
        idx.i = delta_idx;
        idx.j = theta_idx;
        idx.d = dist;
        return idx;
    }
    void ptcl_callback(const sensor_msgs::PointCloud2::ConstPtr &ptcl_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat temp_img = cv::Mat(height, width, CV_32FC1, cv::Scalar(void_value));
        pcl::fromROSMsg(*ptcl_msg, pcl);
        pcl::PointXYZ point;
        for (int i = 0; i < pcl.points.size(); i++)
        {
            point = pcl[i];
            ijd idx = LidarToImageNode::point_to_index(point);
            float distance = idx.d;
            float img_number = distance < max_distance ? distance : max_distance;
            if (invert_distance)
            {
                img_number = max_distance - img_number;
            }
            if (normalize_image)
            {
                img_number /= max_distance;
            }
            if (idx.i < height and idx.j < width)
            {
                temp_img.at<float>(idx.i, idx.j, 0) = img_number;
            }
        }
        sensor_msgs::Image ros_image;
        image.image = temp_img;
        image.toImageMsg(ros_image);
        ros_image.header = ptcl_msg->header;
        ros_image.encoding = "32FC1";
        image_publisher.publish(ros_image);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_to_voxelgrid");
    LidarToImageNode my_class = LidarToImageNode();
    ros::spin();
}