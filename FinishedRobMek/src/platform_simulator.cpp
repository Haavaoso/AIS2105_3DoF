#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlatformSimulator : public rclcpp::Node
{
public:
    PlatformSimulator()
    : Node("3dof_platform")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "physical_model/pose", 10, std::bind(&PlatformSimulator::pose_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("3dof_platform/pose", 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformSimulator>());
    rclcpp::shutdown();
    return 0;
}

