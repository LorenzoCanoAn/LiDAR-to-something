#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>

struct Coordinates
{
    float xyz[3];
};

class LidarToVoxelGridNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher voxel_grid_publisher;
    ros::Subscriber lidar_subscriber;
    std::string input_topic, output_topic;
    float voxel_size;
    float max_x, max_y, max_z;
    int length, width, height, size;
    int *voxel_grid;

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
        size = length * width * height;
        voxel_grid = new int[size];
        lidar_subscriber = nh.subscribe<sensor_msgs::PointCloud>(input_topic, 10, ptcl_callback);
    }
    int coordinates_to_index(float x, float y, float z)
    {
        int i = std::floor((x + max_x)/voxel_size);
        int j = std::floor((y + max_y)/voxel_size);
        int k = std::floor((z + max_z)/voxel_size);
        return i + j*length + k * (length*width);
    }
    void ptcl_callback(sensor_msgs::PointCloud ptcl_msg)
    {
        int n_point = ptcl_msg.points.size();
        geometry_msgs::Point32 point;
        for (int i = 0; i++;i<n_point){
            point = ptcl_msg.points[i];
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    LidarToVoxelGridNode my_class = LidarToVoxelGridNode();
}