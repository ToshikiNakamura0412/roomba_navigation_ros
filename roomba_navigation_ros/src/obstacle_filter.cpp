/**
 * @file obstacle_filter.cpp
 * @author Toshiki Nakamura
 * @brief Obstacle filter node
 * @copyright Copyright (c) 2024
 */

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * @class ObstacleFilter
 * @brief Obstacle filter class
 */
class ObstacleFilter
{
public:
  /**
   * @brief Construct a new Obstacle Filter object
   */
  ObstacleFilter(void) : private_nh_("~")
  {
    private_nh_.param<std::vector<float>>("valid_angle_range_list", valid_angle_range_list_, {-M_PI / 2, M_PI / 2});
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
    cloud_sub_ = nh_.subscribe("/cloud_in", 1, &ObstacleFilter::cloud_callback, this);

    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO("valid_angle_range_list:");
    for (int i = 0; i < valid_angle_range_list_.size(); i += 2)
    {
      ROS_INFO_STREAM("  " << valid_angle_range_list_[i] << ", " << valid_angle_range_list_[i + 1]);
    }
  }

private:
  /**
   * @brief Callback function for the point cloud
   *
   * @param msg Point cloud message
   */
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    sensor_msgs::PointCloud2 filtered_cloud = filter_cloud(*msg);
    filtered_cloud.header = msg->header;
    cloud_pub_.publish(filtered_cloud);
  }

  /**
   * @brief Filter the point cloud
   *
   * @param cloud Point cloud
   * @return sensor_msgs::PointCloud2 Filtered point cloud
   */
  sensor_msgs::PointCloud2 filter_cloud(const sensor_msgs::PointCloud2 &cloud)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in;
    pcl::fromROSMsg(cloud, pcl_cloud_in);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_out;
    for (const auto &point : pcl_cloud_in)
    {
      if (is_valid_point(point))
        pcl_cloud_out.points.push_back(point);
    }
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(pcl_cloud_out, cloud_out);
    return cloud_out;
  }

  /**
   * @brief Check if the point is valid
   *
   * @param point Point
   * @return true If the point is valid
   * @return false If the point is invalid
   */
  bool is_valid_point(const pcl::PointXYZ &point)
  {
    const float angle = atan2(point.y, point.x);
    for (int i = 0; i < valid_angle_range_list_.size(); i += 2)
    {
      if (valid_angle_range_list_[i] <= angle && angle <= valid_angle_range_list_[i + 1])
        return true;
    }
    return false;
  }

  std::vector<float> valid_angle_range_list_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "obstacle_filter");
  ObstacleFilter obstacle_filter;
  ros::spin();

  return 0;
}
