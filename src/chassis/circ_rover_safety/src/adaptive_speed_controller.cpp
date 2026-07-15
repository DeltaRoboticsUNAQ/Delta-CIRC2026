#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <deque>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <mutex>

class AdaptiveSpeedController : public rclcpp::Node
{
public:
    AdaptiveSpeedController() : Node("adaptive_speed_controller"),
                                min_distance_(999.0),
                                current_pitch_(0.0),
                                current_roll_(0.0),
                                z_accel_variance_(0.0)
    {
        this->declare_parameter("warning_distance",      2.0);
        this->declare_parameter("critical_distance",     0.6);
        this->declare_parameter("max_incline_deg",      18.0);
        this->declare_parameter("vibration_buffer_size", 50);
        this->declare_parameter("max_vibration_variance", 4.0);

        // FIX 1: Parámetros para corredor frontal (igual que AEB)
        this->declare_parameter("front_angle_span_deg", 60.0);
        this->declare_parameter("robot_width_m",         0.6);

        // FIX 5: Valor de gravedad en reposo para centrar aceleración Z
        this->declare_parameter("z_gravity_offset",      9.81);

        auto sensor_qos = rclcpp::SensorDataQoS();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sensor_qos,
            std::bind(&AdaptiveSpeedController::scan_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", sensor_qos,
            std::bind(&AdaptiveSpeedController::imu_callback, this, std::placeholders::_1));

        // FIX 1: Input ahora viene del raw, output a tópico intermedio
        cmd_vel_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10,
            std::bind(&AdaptiveSpeedController::cmd_vel_raw_callback, this, std::placeholders::_1));

        // FIX 1: Publica a tópico intermedio, no a /cmd_vel_safe
        // La cadena es: /cmd_vel_raw -> adaptive -> /cmd_vel_adaptive -> AEB -> /cmd_vel_safe
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_adaptive", 10);

        RCLCPP_INFO(this->get_logger(), "Adaptive Speed Controller iniciado.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;

    // FIX 3: Mutex para proteger variables compartidas entre callbacks
    std::mutex sensor_mutex_;

    double min_distance_;
    double current_pitch_;
    double current_roll_;
    double z_accel_variance_;
    std::deque<double> z_accel_buffer_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // FIX 2: Solo corredor frontal, no distancia global
        double span_deg   = this->get_parameter("front_angle_span_deg").as_double();
        double robot_width = this->get_parameter("robot_width_m").as_double();
        double span_rad   = span_deg * M_PI / 180.0;

        double current_min = 999.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;

            // Solo sector frontal
            if (std::abs(angle) >= (span_rad / 2.0)) continue;

            // Solo dentro del ancho del corredor del rover
            double y_pos = std::abs(r * std::sin(angle));
            if (y_pos >= (robot_width / 2.0 + 0.1)) continue;

            double x_proj = r * std::cos(angle);
            if (x_proj < current_min) current_min = x_proj;
        }

        std::lock_guard<std::mutex> lock(sensor_mutex_);  // FIX 3
        min_distance_ = current_min;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Euler angles desde cuaternión
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // FIX 5: Centrar la aceleración Z restando la gravedad en reposo
        double z_gravity = this->get_parameter("z_gravity_offset").as_double();
        double z_centered = msg->linear_acceleration.z - z_gravity;

        int buf_size = this->get_parameter("vibration_buffer_size").as_int();

        // Calcular varianza antes de tomar el lock (operación costosa)
        std::deque<double> local_buf;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);  // FIX 3
            z_accel_buffer_.push_back(z_centered);
            if ((int)z_accel_buffer_.size() > buf_size) z_accel_buffer_.pop_front();
            local_buf = z_accel_buffer_; // copia local para calcular fuera del lock
        }

        double variance = 0.0;
        if (local_buf.size() > 1)
        {
            double sum  = std::accumulate(local_buf.begin(), local_buf.end(), 0.0);
            double mean = sum / local_buf.size();

            double sq_sum = std::inner_product(
                local_buf.begin(), local_buf.end(),
                local_buf.begin(), 0.0,
                std::plus<double>(),
                [mean](double a, double b) { return (a - mean) * (b - mean); });

            variance = sq_sum / (local_buf.size() - 1);
        }

        std::lock_guard<std::mutex> lock(sensor_mutex_);  // FIX 3
        current_roll_      = roll;
        current_pitch_     = pitch;
        z_accel_variance_  = variance;
    }

    void cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double warn_dist  = this->get_parameter("warning_distance").as_double();
        double crit_dist  = this->get_parameter("critical_distance").as_double();
        double max_inc_rad = this->get_parameter("max_incline_deg").as_double() * (M_PI / 180.0);
        double max_vib    = this->get_parameter("max_vibration_variance").as_double();

        // Leer estado de sensores de forma segura
        double min_dist, pitch, roll, variance;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);  // FIX 3
            min_dist = min_distance_;
            pitch    = current_pitch_;
            roll     = current_roll_;
            variance = z_accel_variance_;
        }

        // --- Factor 1: Proximidad ---
        double S_obs = 1.0;
        if (min_dist <= crit_dist) {
            S_obs = 0.0;
        } else if (min_dist < warn_dist) {
            S_obs = (min_dist - crit_dist) / (warn_dist - crit_dist);
        }

        // --- Factor 2: Inclinación ---
        double S_inc = 1.0;
        double max_current_inc = std::max(std::abs(pitch), std::abs(roll));
        if (max_current_inc > max_inc_rad) {
            S_inc = 0.0;
        } else if (max_current_inc > (max_inc_rad * 0.5)) {
            S_inc = 1.0 - ((max_current_inc - (max_inc_rad * 0.5)) / (max_inc_rad * 0.5));
        }

        // --- Factor 3: Vibración ---
        double S_vib = 1.0;
        if (variance > max_vib) {
            S_vib = 0.2;
        } else if (variance > (max_vib * 0.3)) {
            S_vib = 1.0 - (0.8 * ((variance - (max_vib * 0.3)) / (max_vib * 0.7)));
        }

        double safety_multiplier = std::clamp(S_obs * S_inc * S_vib, 0.0, 1.0);

        geometry_msgs::msg::Twist safe_msg = *msg;

        // FIX 4: Solo modular avance frontal; reversa pasa libre
        if (msg->linear.x > 0.0) {
            safe_msg.linear.x = msg->linear.x * safety_multiplier;
        }
        // linear.x < 0 (reversa): sin restricción por obstáculo frontal

        safe_msg.linear.y  *= safety_multiplier;
        safe_msg.angular.z *= safety_multiplier;

        cmd_vel_pub_->publish(safe_msg);

        if (safety_multiplier < 0.9 && std::abs(msg->linear.x) > 0.01) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Seguridad] Modulando vel: (Obs: %.2fm | Inc: %.1fdeg | Vib: %.3f) -> Mul: %.2f",
                min_dist, max_current_inc * (180.0 / M_PI), variance, safety_multiplier);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveSpeedController>());
    rclcpp::shutdown();
    return 0;
}