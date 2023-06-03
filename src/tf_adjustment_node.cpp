#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>

class TFAdjustmentNode : public rclcpp::Node
{
public:
    TFAdjustmentNode() : Node("tf_adjustment_node")
    {
        // Lấy các tham số từ file launch
        double x_translation, y_translation, z_translation;
        double roll_rotation, pitch_rotation, yaw_rotation;
        this->declare_parameter("x_translation", 0.0);
        this->declare_parameter("y_translation", 0.0);
        this->declare_parameter("z_translation", 0.0);
        this->declare_parameter("roll_rotation", 0.0);
        this->declare_parameter("pitch_rotation", 0.0);
        this->declare_parameter("yaw_rotation", 0.0);
        this->get_parameter("x_translation", x_translation);
        this->get_parameter("y_translation", y_translation);
        this->get_parameter("z_translation", z_translation);
        this->get_parameter("roll_rotation", roll_rotation);
        this->get_parameter("pitch_rotation", pitch_rotation);
        this->get_parameter("yaw_rotation", yaw_rotation);

        // Tạo transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Tạo transform từ base_link đến camera_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "camera_link";
        transform_stamped.transform.translation.x = x_translation;
        transform_stamped.transform.translation.y = y_translation;
        transform_stamped.transform.translation.z = z_translation;
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll_rotation, pitch_rotation, yaw_rotation);
        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();

        // Phát transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFAdjustmentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
