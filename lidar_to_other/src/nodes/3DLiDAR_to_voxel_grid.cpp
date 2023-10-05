#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>

class LidarToVoxelGridNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher voxel_grid_publisher;
    ros::Subscriber lidar_subscriber;
    std::string input_topic, output_topic;
    float voxel_size;
    float max_x, max_y, max_z;
    float x_offset, y_offset, z_offset;
    int length, width,height;
public:
    LidarToVoxelGridNode(void)
    {
        nh = ros::NodeHandle("~");
        nh.param<std::string>("default_param", input_topic, "lidar_reading");
        nh.param<std::string>("output_topic", output_topic, "voxelized_lidar");
        nh.param<float>("voxel_size", voxel_size, 1);
        nh.param<float>("max_x", max_x, 10);
        nh.param<float>("max_y", max_y, 10);
        nh.param<float>("max_z", max_z, 3);
        length = std::floor(max_x * 2 / voxel_size);
        width = std::floor(max_y * 2 / voxel_size);
        height = std::floor(max_z * 2 / voxel_size);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    LidarToVoxelGridNode my_class = LidarToVoxelGridNode();
}