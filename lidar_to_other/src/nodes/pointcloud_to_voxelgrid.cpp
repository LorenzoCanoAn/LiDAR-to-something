#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <chrono>
struct Coordinats
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
    pcl::PointCloud<pcl::PointXYZ> pcl;
    float voxel_size;
    float max_x, max_y, max_z;
    int length, width, height, size;
    int *voxel_grid;

public:
    LidarToVoxelGridNode(void)
    {
        nh = ros::NodeHandle("~");
        nh.param<std::string>("input_topic", input_topic, "lidar_reading");
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
        ROS_DEBUG("Input topic:  %s", input_topic.data());
        ROS_DEBUG("Output topic: %s", output_topic.data());
        ROS_DEBUG("Voxel size: %f", voxel_size);
        ROS_DEBUG("Max x, y, z:(%f,%f,%f)", max_x, max_y, max_z);
        ROS_DEBUG("Voxel grid size: %i", size);
        lidar_subscriber = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &LidarToVoxelGridNode::ptcl_callback, this);
    }
    bool is_point_in_grid(pcl::PointXYZ pt){
        return is_point_in_grid(pt.x, pt.y, pt.z);
    }
    bool is_point_in_grid(float x, float y, float z)
    {
        return std::abs(x) < this->max_x and std::abs(y) < this->max_y and std::abs(y) < this->max_y;
    }
    int point_to_index(pcl::PointXYZ pt){
        return coordinates_to_index(pt.x, pt.y, pt.z);
    }
    int coordinates_to_index(float x, float y, float z)
    {
        int i = std::floor((x + max_x) / voxel_size);
        int j = std::floor((y + max_y) / voxel_size);
        int k = std::floor((z + max_z) / voxel_size);
        ROS_DEBUG("XYZ: (%i, %i, %i)", i, j, k);
        ROS_DEBUG("LENGTH: (%i)", length);
        ROS_DEBUG("WIDTH: (%i)",width);
        int idx =i + j * length + k * (length * width);
        ROS_DEBUG("IDX: (%i)",idx);
        return idx;
    }
    void ptcl_callback(const sensor_msgs::PointCloud2::ConstPtr &ptcl_msg)
    {
        // reset voxel grid
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < this->size; i++)
        {
            this->voxel_grid[i] = 0;
        }
        pcl::fromROSMsg(*ptcl_msg, pcl);
        pcl::PointXYZ point;
        int max_size = 0;
        for (int i = 0; i < pcl.width; i++)
        {
            point = pcl[i];
            int idx = LidarToVoxelGridNode::point_to_index(point);
            if (idx > this->size)
            {
                ROS_WARN("Idx too large, the max is %i but %i is obtained", this->size - 1, idx);
            }
            this->voxel_grid[idx] += 1;
        }
        ROS_INFO("Max idx: %i ", max_size);
        int time_before_loop_begins = std::time(NULL);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // ROS_INFO("Callback duration: %i microseconds",duration.count());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_to_voxelgrid");
    LidarToVoxelGridNode my_class = LidarToVoxelGridNode();
    ros::spin();
}