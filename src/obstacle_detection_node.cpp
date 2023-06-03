#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/statistical_outlier_removal.h>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
      : Node("obstacle_detection_node")
  {
    // Subscribe to the input point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10,
        std::bind(&ObstacleDetectionNode::processPointCloud, this, std::placeholders::_1));

    // Initialize publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_point_cloud_topic",
        10);

    // Create publishers for the processed point clouds
    filtered_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "filtered_point_cloud_topic", 10);
    downsampled_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "downsampled_point_cloud_topic", 10);
    sor_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "StatisticalOutlierRemoval_point_cloud_topic", 10);
    obstacles_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "obstacles_point_cloud_topic", 10);
    clustered_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "clustered_point_cloud_topic", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS PointCloud2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Step 1: PassThrough filter to remove irrelevant points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    pass.filter(*cloud_filtered);

    // Publish the filtered point cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header = msg->header;
    filtered_publisher_->publish(filtered_msg);

    // Step 2: VoxelGrid filter to downsample the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud_filtered);
    voxelGrid.setLeafSize(0.02, 0.02, 0.02);
    voxelGrid.filter(*cloud_downsampled);

    // Publish the downsampled point cloud
    sensor_msgs::msg::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*cloud_downsampled, downsampled_msg);
    downsampled_msg.header = msg->header;
    downsampled_publisher_->publish(downsampled_msg);

    // Apply StatisticalOutlierRemoval filter to remove outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_downsampled);
    sor.setMeanK(50);            // Number of nearest neighbors to compute mean distance
    sor.setStddevMulThresh(1.0); // Standard deviation multiplier threshold
    sor.filter(*cloud_sor);

    // Publish filtered point cloud
    sensor_msgs::msg::PointCloud2 sor_msg;
    pcl::toROSMsg(*cloud_sor, sor_msg);
    sor_publisher_->publish(sor_msg);

    // Step 3: RANSAC segmentation to separate ground plane from obstacles
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud_downsampled);
    seg.setModelType(pcl::SACMODEL_PLANE); // xác định mô hình SACMODEL_PLANE là xác định mặt phẳng
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(400);
    seg.setDistanceThreshold(0.01); // Adjust the distance threshold as needed
    seg.segment(*inliers, *coefficients);

    // Extract the obstacles by removing the ground plane
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_downsampled);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obstacles);

    // Publish the obstacles point cloud
    sensor_msgs::msg::PointCloud2 obstacles_msg;
    pcl::toROSMsg(*cloud_obstacles, obstacles_msg);
    obstacles_msg.header = msg->header;
    obstacles_publisher_->publish(obstacles_msg);

    // Step 4: Euclidean clustering to group obstacles
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_obstacles);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // Adjust the cluster tolerance as needed
    ec.setMinClusterSize(50);    // Adjust the minimum cluster size as needed
    ec.setMaxClusterSize(10000);  // Adjust the maximum cluster size as needed
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obstacles);
    ec.extract(cluster_indices);

    // Publish the clustered point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &indices : cluster_indices)
    {
      for (const auto &index : indices.indices)
      {
        pcl::PointXYZRGB point;
        point.x = cloud_obstacles->points[index].x;
        point.y = cloud_obstacles->points[index].y;
        point.z = cloud_obstacles->points[index].z;
        point.r = 255;
        point.g = 0;
        point.b = 0;
        cloud_clustered->points.push_back(point);
      }
    }
    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;
    cloud_clustered->is_dense = true;

    sensor_msgs::msg::PointCloud2 clustered_msg;
    pcl::toROSMsg(*cloud_clustered, clustered_msg);
    clustered_msg.header = msg->header;
    clustered_publisher_->publish(clustered_msg);

    // Publish transform from camera_link to base_link
    pcl::PointXYZ camera_translation(0.0, 0.0, 0.2); // Modify the camera translation according to the camera position
    tf2::Quaternion camera_rotation;
    camera_rotation.setRPY(-1.5708, 0, 0); // Modify the rotation angles according to the camera orientation
    publishTransform("base_link", "camera_link", camera_translation, camera_rotation);
  }
  void publishTransform(const std::string &frame_id, const std::string &child_frame_id,
                        const pcl::PointXYZ &translation, const tf2::Quaternion &rotation)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform.translation.x = translation.x;
    transform_stamped.transform.translation.y = translation.y;
    transform_stamped.transform.translation.z = translation.z;
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sor_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
