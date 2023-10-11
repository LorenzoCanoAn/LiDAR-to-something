#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <string>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <voxelgrid_msgs/VoxelGridFloat32MultiarrayStamped.h>
#include <voxelgrid_msgs/VoxelGridInt16MultiarrayStamped.h>
struct Coordinats
{
    float xyz[3];
};

class LidarToVoxelGridNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher voxelgrid_publisher;
    ros::Subscriber lidar_subscriber;
    std::string input_topic, output_topic, visualization_topic, int_topic, float_topic;
    pcl::PointCloud<pcl::PointXYZ> pcl;
    voxelgrid_msgs::VoxelGridFloat32MultiarrayStamped float_voxelgrid_msg;
    //voxelgrid_msgs::VoxelGridInt16MultiarrayStamped int_voxelgrid_msg;
    float voxel_size;
    float max_x, max_y, max_z;
    int x_size, y_size, z_size, array_size;
    std::string frame;

public:
    LidarToVoxelGridNode(void)
    {
        nh = ros::NodeHandle("~");
        nh.param<std::string>("input_topic", input_topic, "lidar_reading");
        nh.param<std::string>("visualization_voxel_grid_topic", visualization_topic, "voxel_grid_rviz");
        nh.param<std::string>("int_voxel_grid_topic", int_topic, "voxel_grid_int");
        nh.param<std::string>("float_voxel_grid_topic", float_topic, "voxelized_lidar");
        nh.param<float>("voxel_size", voxel_size, 1);
        nh.param<float>("max_x", max_x, 5);
        nh.param<float>("max_y", max_y, 5);
        nh.param<float>("max_z", max_z, 2);
        nh.param<std::string>("frame", frame, "voxelgrid_frame");
        x_size = std::floor(max_x * 2 / voxel_size);
        y_size = std::floor(max_y * 2 / voxel_size);
        z_size = std::floor(max_z * 2 / voxel_size);
        array_size = x_size * y_size * z_size;
        std_msgs::MultiArrayDimension x_dim;
        std_msgs::MultiArrayDimension y_dim;
        std_msgs::MultiArrayDimension z_dim;
        x_dim.label = "x";
        y_dim.label = "y";
        z_dim.label = "z";
        x_dim.size = x_size;
        y_dim.size = y_size;
        z_dim.size = z_size;
        x_dim.stride = y_size * z_size;
        y_dim.stride = z_size;
        z_dim.stride = 1;
        float_voxelgrid_msg.voxel_grid.array.data = std::vector<float>(array_size, 0);
        float_voxelgrid_msg.voxel_grid.array.layout.dim.push_back(x_dim);
        float_voxelgrid_msg.voxel_grid.array.layout.dim.push_back(y_dim);
        float_voxelgrid_msg.voxel_grid.array.layout.dim.push_back(z_dim);
        float_voxelgrid_msg.header.frame_id = frame;
        float_voxelgrid_msg.voxel_grid.origin.x = -max_x;
        float_voxelgrid_msg.voxel_grid.origin.y = -max_y;
        float_voxelgrid_msg.voxel_grid.origin.z = -max_z;
        float_voxelgrid_msg.voxel_grid.scale.data = voxel_size;
        ROS_DEBUG("Input topic:  %s", input_topic.data());
        ROS_DEBUG("Output topic: %s", output_topic.data());
        ROS_DEBUG("Voxel size: %f", voxel_size);
        ROS_DEBUG("Max x, y, z:(%f,%f,%f)", max_x, max_y, max_z);
        ROS_DEBUG("Voxel grid size: %i", array_size);
        lidar_subscriber = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, &LidarToVoxelGridNode::ptcl_callback, this);
        voxelgrid_publisher = nh.advertise<voxelgrid_msgs::VoxelGridFloat32MultiarrayStamped>("voxel_grid", 1);
    }
    bool is_point_in_grid(pcl::PointXYZ pt)
    {
        return is_point_in_grid(pt.x, pt.y, pt.z);
    }
    bool is_point_in_grid(float x, float y, float z)
    {
        return std::abs(x) < this->max_x and std::abs(y) < this->max_y and std::abs(y) < this->max_y;
    }
    int point_to_index(pcl::PointXYZ pt)
    {
        return coordinates_to_index(pt.x, pt.y, pt.z);
    }
    int coordinates_to_index(float x, float y, float z)
    {
        int i = std::floor((x + max_x) / voxel_size);
        int j = std::floor((y + max_y) / voxel_size);
        int k = std::floor((z + max_z) / voxel_size);
        int idx = i * y_size * z_size + j * z_size + k;
        return idx;
    }
    void ptcl_callback(const sensor_msgs::PointCloud2::ConstPtr &ptcl_msg)
    {
        // reset voxel grid
        ROS_DEBUG("Callback start");
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < this->array_size; i++)
        {
            float_voxelgrid_msg.voxel_grid.array.data[i] = 0.0;
        }
        pcl::fromROSMsg(*ptcl_msg, pcl);
        pcl::PointXYZ point;
        for (int i = 0; i < pcl.width; i++)
        {
            point = pcl[i];
            if (is_point_in_grid(point))
            {
                int idx = LidarToVoxelGridNode::point_to_index(point);
                float_voxelgrid_msg.voxel_grid.array.data[idx] = 1.0;
            }
        }
        this->float_voxelgrid_msg.header.stamp = ros::Time::now();
        voxelgrid_publisher.publish(float_voxelgrid_msg);
        auto end = std::chrono::high_resolution_clock::now();
        auto msecs = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        ROS_DEBUG("Callback time: %li", msecs.count());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_to_voxelgrid");
    LidarToVoxelGridNode my_class = LidarToVoxelGridNode();
    ros::spin();
}