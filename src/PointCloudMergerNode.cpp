#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudMergerNode : public rclcpp::Node
{
public:
  PointCloudMergerNode()
    : Node("point_cloud_merger_node")
  {
    merged_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", 10);

    // Subscribe to the two point cloud topics
    subscriber1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera1/depth/color/points", 10, std::bind(&PointCloudMergerNode::pointCloudCallback1, this, std::placeholders::_1));
    subscriber2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera2/depth/color/points", 10, std::bind(&PointCloudMergerNode::pointCloudCallback2, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    merged_cloud_ = *msg;

    // Process the point cloud data from camera1

    // Publish the merged point cloud
    merged_publisher_->publish(merged_cloud_);
  }

  void pointCloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    merged_cloud_ = *msg;

    // Process the point cloud data from camera2

    // Publish the merged point cloud
    merged_publisher_->publish(merged_cloud_);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber2_;

  sensor_msgs::msg::PointCloud2 merged_cloud_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudMergerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
