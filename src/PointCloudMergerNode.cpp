#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudMergerNode : public rclcpp::Node
{
public:
  PointCloudMergerNode()
    : Node("point_cloud_merger_node")
  {
    // Khởi tạo publisher để gửi merged point cloud
    merged_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", 30);

    // Khởi tạo subscriber cho camera1 point cloud
    camera1_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera1/depth/color/points", 30, std::bind(&PointCloudMergerNode::camera1Callback, this, std::placeholders::_1));

    // Khởi tạo subscriber cho camera2 point cloud
    camera2_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera2/depth/color/points", 30, std::bind(&PointCloudMergerNode::camera2Callback, this, std::placeholders::_1));
  }

private:
  void camera1Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Sao chép dữ liệu từ camera1 point cloud vào merged point cloud
    merged_cloud_ = *msg;

    // Gửi merged point cloud qua publisher
    merged_publisher_->publish(merged_cloud_);
  }

  void camera2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Gộp dữ liệu từ camera2 point cloud vào merged point cloud
    merged_cloud_.data.insert(merged_cloud_.data.end(), msg->data.begin(), msg->data.end());
    merged_cloud_.width += msg->width;
    merged_cloud_.row_step += msg->row_step;

    // Gửi merged point cloud qua publisher
    merged_publisher_->publish(merged_cloud_);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera1_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera2_subscription_;
  sensor_msgs::msg::PointCloud2 merged_cloud_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudMergerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
