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
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <std_msgs/msg/string.hpp>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
      : Node("obstacle_detection_node")
  {
    // Subscribe to the input point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 30,
        std::bind(&ObstacleDetectionNode::processPointCloud, this, std::placeholders::_1));


    // Create publishers for the processed point clouds
    filtered_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_publisher", 30);
    downsampled_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("downsampled_publisher", 30);
    //denoised_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("StatisticalOutlierRemoval_point_cloud_topic", 30);
    obstacles_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles_publisher", 30);
    clustered_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_publisher", 30);
    warning_objects_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("warning_objects_publisher", 30);
    protection_objects_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("protection_objects_publisher", 30);
    safety_status_publisher_ = this->create_publisher<std_msgs::msg::String>("safety_status_publisher", 30);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    captured_object_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    warning_zone_limit = 1.0;  // Adjust the warning zone limit as needed
    protection_zone_limit = 0.5;  // Adjust the protection zone limit as needed


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
    // filter z
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 5.0);
    pass.filter(*cloud_filtered);
    // filter y
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5, 2);
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
    voxelGrid.setLeafSize(0.01, 0.01, 0.01);
    voxelGrid.filter(*cloud_downsampled);

    // Publish the downsampled point cloud
    sensor_msgs::msg::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*cloud_downsampled, downsampled_msg);
    downsampled_msg.header = msg->header;
    downsampled_publisher_->publish(downsampled_msg);

    // // Apply StatisticalOutlierRemoval filter to remove outliers
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_denoised(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud_downsampled);
    // sor.setMeanK(50);            // Number of nearest neighbors to compute mean distance
    // sor.setStddevMulThresh(1.0); // Standard deviation multiplier threshold
    // sor.filter(*cloud_denoised);

    // // Publish filtered point cloud
    // sensor_msgs::msg::PointCloud2 denoised_msg;
    // pcl::toROSMsg(*cloud_denoised, denoised_msg);
    // denoised_publisher_->publish(denoised_msg);

    // Step 3: RANSAC segmentation to separate ground plane from obstacles
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

    // Extract ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_downsampled);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground_plane);

    // Remove ground plane from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*cloud_obstacles);

    // Publish the obstacles point cloud
    sensor_msgs::msg::PointCloud2 obstacles_msg;
    pcl::toROSMsg(*cloud_obstacles, obstacles_msg);
    obstacles_msg.header = msg->header;
    obstacles_publisher_->publish(obstacles_msg);

    // Step 4: Euclidean clustering to group obstacles
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(cloud_obstacles);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // Adjust the cluster tolerance as needed
    ec.setMinClusterSize(50);    // Adjust the minimum cluster size as needed
    ec.setMaxClusterSize(10000);  // Adjust the maximum cluster size as needed
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(cloud_obstacles);
    ec.extract(cluster_indices);

    // Publish the clustered point cloud
    // Iterate through each cluster
    for (const auto& indices : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& index : indices.indices)
      {
        cloud_clustered->points.push_back(cloud_obstacles->points[index]);
      }
      cloud_clustered->width = cloud_clustered->points.size();
      cloud_clustered->height = 1;
      cloud_clustered->is_dense = true;

    sensor_msgs::msg::PointCloud2 clustered_msg;
    pcl::toROSMsg(*cloud_clustered, clustered_msg);
    clustered_msg.header = msg->header;
    clustered_publisher_->publish(clustered_msg);


    // Publish the clustered point clouds and detect objects in safety zones
    bool warning_zone_detected = false;
    bool protection_zone_detected = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr warning_objects_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr protection_objects_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// Compute the centroid of the cluster
      pcl::PointXYZ centroid;
      pcl::computeCentroid(*cloud_clustered, centroid);

      // Check if the cluster falls within the warning zone
      if (centroid.x < warning_zone_limit && centroid.y < warning_zone_limit && centroid.z < warning_zone_limit)
      {
        warning_zone_detected = true;
        *warning_objects_cloud += *cloud_clustered;
      }

      // Check if the cluster falls within the protection zone
      if (centroid.x < protection_zone_limit && centroid.y < protection_zone_limit && centroid.z < protection_zone_limit)
      {
        protection_zone_detected = true;
        *protection_objects_cloud += *cloud_clustered;
      }

      /// Publish the warning objects point cloud
    sensor_msgs::msg::PointCloud2 warning_objects_msg;
    pcl::toROSMsg(*warning_objects_cloud, warning_objects_msg);
    warning_objects_msg.header = msg->header;
    warning_objects_publisher_->publish(warning_objects_msg);

    // Publish the protection objects point cloud
    sensor_msgs::msg::PointCloud2 protection_objects_msg;
    pcl::toROSMsg(*protection_objects_cloud, protection_objects_msg);
    protection_objects_msg.header = msg->header;
    protection_objects_publisher_->publish(protection_objects_msg);

    // Publish the safety status
    std_msgs::msg::String safety_status_msg;
    if (protection_zone_detected)
    {
      safety_status_msg.data = "Danger";
    }
    else if (warning_zone_detected)
    {
      safety_status_msg.data = "Warning";
    }
    else
    {
      safety_status_msg.data = "Safe";
    }
    safety_status_publisher_->publish(safety_status_msg);
 
        // Set the initial transformation from camera_link to base_link
    pcl::PointXYZ translation(0.0, 0.0, 0.2);
    tf2::Quaternion rotation;
    // rotation.setRPY(-1.5708, 0, 0);
   // rotation.setRPY(0, 0.17453293, 0);
   rotation.setRPY(-1.5708, 0, 0);
    publishTransform("base_link", "camera_base", translation, rotation);

    // Perform additional processing to detect motion in warning and protection zones
     // detectMotionInZones(cloud_clustered);
  }
  }
  void detectMotionInZones(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_clustered)
  {
    // Define the warning zone dimensions and distance thresholds
    double warning_zone_x_min = -0.5;    // Modify the warning zone x min value as needed
    double warning_zone_x_max = 0.5;     // Modify the warning zone x max value as needed
    double warning_zone_y_min = -0.1;    // Modify the warning zone y min value as needed
    double warning_zone_y_max = 0.1;     // Modify the warning zone y max value as needed
    double warning_distance_threshold = 0.5;   // Modify the warning distance threshold as needed

    // Define the protection zone dimensions and distance thresholds
    double protection_zone_x_min = -0.25;    // Modify the protection zone x min value as needed
    double protection_zone_x_max = 0.25;     // Modify the protection zone x max value as needed
    double protection_zone_y_min = -0.1;    // Modify the protection zone y min value as needed
    double protection_zone_y_max = 0.1;     // Modify the protection zone y max value as needed
    double protection_distance_threshold = 0.3;   // Modify the protection distance threshold as needed

    // Calculate the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_clustered, centroid);

    // Check if the centroid is within the warning zone
    if (centroid[0] >= warning_zone_x_min && centroid[0] <= warning_zone_x_max &&
        centroid[1] >= warning_zone_y_min && centroid[1] <= warning_zone_y_max)
    {
      // Calculate the distance from the centroid to the origin (base_link)
      double distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);

      // Check if the distance is less than the warning distance threshold
      if (distance <= warning_distance_threshold)
      {
        // Publish a warning message
        RCLCPP_WARN(this->get_logger(), "Object detected in warning zone");
        // Perform additional actions as needed
      }
    }

    // Check if the centroid is within the protection zone
    if (centroid[0] >= protection_zone_x_min && centroid[0] <= protection_zone_x_max &&
        centroid[1] >= protection_zone_y_min && centroid[1] <= protection_zone_y_max)
    {
      // Calculate the distance from the centroid to the origin (base_link)
      double distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);

      // Check if the distance is less than the protection distance threshold
      if (distance <= protection_distance_threshold)
      {
        // Publish an error message
        RCLCPP_ERROR(this->get_logger(), "Object detected in protection zone! Emergency stop!");
        // Perform additional actions as needed
      }
    }
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
  //::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr denoised_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr warning_objects_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr protection_objects_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_publisher_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr captured_object_cloud_;
  

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double warning_zone_limit;
  double protection_zone_limit;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}