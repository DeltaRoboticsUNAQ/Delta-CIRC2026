#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>

class XboxTeleop : public rclcpp::Node
{
public:
    XboxTeleop() : Node("xbox_teleop")
    {
        this->declare_parameter("max_linear_speed",  1.5);  // m/s
        this->declare_parameter("max_angular_speed", 2.0);  // rad/s
        this->declare_parameter("axis_linear",       1);    // Joystick izq. Arriba/Abajo
        this->declare_parameter("axis_angular",      0);    // Joystick izq. Izq/Der
        this->declare_parameter("enable_button",     4);    // LB = mantener para mover
        this->declare_parameter("deadzone",          0.1);  // ruido del joystick

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&XboxTeleop::joy_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);

        RCLCPP_INFO(this->get_logger(), "Xbox Teleop iniciado. Mantén LB para mover el rover.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        int    axis_linear  = this->get_parameter("axis_linear").as_int();
        int    axis_angular = this->get_parameter("axis_angular").as_int();
        int    enable_btn   = this->get_parameter("enable_button").as_int();
        double max_linear   = this->get_parameter("max_linear_speed").as_double();
        double max_angular  = this->get_parameter("max_angular_speed").as_double();
        double deadzone     = this->get_parameter("deadzone").as_double();

        // FIX 2: Deadman switch — si LB no está presionado, detener
        if ((int)msg->buttons.size() <= enable_btn || msg->buttons[enable_btn] == 0) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
            return;
        }

        // FIX 1: Validación correcta de tamaño
        if ((int)msg->axes.size() <= std::max(axis_linear, axis_angular)) {
            RCLCPP_WARN(this->get_logger(), "Ejes del joystick insuficientes.");
            return;
        }

        // FIX 3: Aplicar zona muerta
        auto apply_deadzone = [deadzone](double val) -> double {
            return std::abs(val) < deadzone ? 0.0 : val;
        };

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = apply_deadzone(msg->axes[axis_linear])  * max_linear;
        twist.angular.z = apply_deadzone(msg->axes[axis_angular]) * max_angular;

        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XboxTeleop>());
    rclcpp::shutdown();
    return 0;
}