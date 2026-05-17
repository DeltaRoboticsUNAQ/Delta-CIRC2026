#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <circ_chassis_msgs/msg/aeb_status.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

class AEBNodeC : public rclcpp::Node
{
public:
  AEBNodeC() : Node("aeb_controller"), is_engaged_(false)
  {
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("cmd_topic_in", "/cmd_vel_raw");
    this->declare_parameter("cmd_topic_out", "/cmd_vel_safe");
    this->declare_parameter("status_topic", "/chassis/safety/aeb_status");
    
    this->declare_parameter("ttc_threshold_s", 1.2);
    this->declare_parameter("min_distance_threshold_m", 0.6);
    this->declare_parameter("front_angle_span_deg", 60.0);
    this->declare_parameter("robot_width_m", 0.6);

    // FIX 1: Nuevo parámetro de distancia de frenado
    // A 0.5 m/s necesitas al menos ~0.3m para frenar; ajusta según tu rover
    this->declare_parameter("braking_distance_m", 0.4);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&AEBNodeC::scan_callback, this, std::placeholders::_1));

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_topic_in").as_string(), 10,
      std::bind(&AEBNodeC::cmd_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_topic_out").as_string(), 10);

    status_pub_ = this->create_publisher<circ_chassis_msgs::msg::AEBStatus>(
      this->get_parameter("status_topic").as_string(), 10);
      
    RCLCPP_INFO(this->get_logger(), "AEB C++ Node initialized for safety-critical logic.");
  }

private:
  // FIX 2: cmd_callback ahora usa la misma lógica de filtrado seguro
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    latest_cmd_ = *msg;

    if (!is_engaged_) {
      cmd_pub_->publish(*msg);
    } else {
      // Publica el comando filtrado (reversa/lateral permitidos)
      cmd_pub_->publish(get_safe_cmd(*msg));
    }
  }

  // FIX 3: Función que filtra solo el movimiento peligroso
  // Solo bloquea linear.x hacia adelante; reversa y giro siempre permitidos
  geometry_msgs::msg::Twist get_safe_cmd(const geometry_msgs::msg::Twist & cmd)
  {
    geometry_msgs::msg::Twist safe = cmd;

    // Bloquar avance hacia adelante únicamente
    if (safe.linear.x > 0.0) {
      safe.linear.x = 0.0;
    }

    // linear.y (si tu rover lo soporta), linear.z, angular.z → sin restricción
    // El rover puede retroceder (linear.x < 0) y girar libremente

    return safe;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_distance = std::numeric_limits<double>::infinity();
    double global_min  = std::numeric_limits<double>::infinity();
    
    double ttc_threshold   = this->get_parameter("ttc_threshold_s").as_double();
    double min_dist_thresh = this->get_parameter("min_distance_threshold_m").as_double();
    double span_deg        = this->get_parameter("front_angle_span_deg").as_double();
    double robot_width     = this->get_parameter("robot_width_m").as_double();
    double braking_dist    = this->get_parameter("braking_distance_m").as_double();  // FIX 1

    double span_rad = span_deg * M_PI / 180.0;
    
    double v_x = 0.0;
    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      v_x = latest_cmd_.linear.x;
    }

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      if (r < global_min) global_min = r;

      double angle = msg->angle_min + i * msg->angle_increment;

      if (std::abs(angle) < (span_rad / 2.0)) {
        double y_pos = std::abs(r * std::sin(angle));
        if (y_pos < (robot_width / 2.0 + 0.1)) {
          double x_proj = r * std::cos(angle);
          if (x_proj < min_distance) {
            min_distance = x_proj;
          }
        }
      }
    }

    bool should_engage = false;
    double current_ttc = std::numeric_limits<double>::infinity();
    std::string reason = "Safe";

    // FIX 2: Solo activar AEB si el rover va hacia adelante (v_x > 0)
    // Si va de reversa no tiene sentido frenar por obstáculo frontal
    if (min_distance < msg->range_max && v_x > 0.0)
    {
      // FIX 1: Distancia absoluta mínima + margen de frenado
      // El umbral efectivo es: min_dist + braking_dist
      double effective_threshold = min_dist_thresh + braking_dist;

      if (min_distance <= effective_threshold) {
        should_engage = true;
        reason = "Obstacle within braking zone (" + std::to_string(min_distance) + "m, threshold=" +
                 std::to_string(effective_threshold) + "m)";
      }
      else if (v_x > 0.05) {
        current_ttc = min_distance / v_x;
        // FIX 1: TTC se extiende para dar margen real de frenado
        double effective_ttc = ttc_threshold + (braking_dist / v_x);
        if (current_ttc <= effective_ttc) {
          should_engage = true;
          reason = "TTC violation (" + std::to_string(current_ttc) +
                   "s, threshold=" + std::to_string(effective_ttc) + "s)";
        }
      }
    }

    if (should_engage && !is_engaged_) {
      RCLCPP_WARN(this->get_logger(), "AEB ENGAGED! Reason: %s", reason.c_str());
    } else if (!should_engage && is_engaged_) {
      RCLCPP_INFO(this->get_logger(), "AEB DISENGAGED. Path is clear.");
    }

    static int debug_counter = 0;
    if (debug_counter++ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "DEBUG: Global Min: %.2f m | Forward Corridor: %.2f m | v_x: %.2f m/s | TTC: %.2f s",
        global_min, min_distance, v_x,
        std::isinf(current_ttc) ? -1.0 : current_ttc);
    }

    is_engaged_ = should_engage;

    // FIX 3: Al engaged, no mandamos Twist vacío sino el comando filtrado
    if (is_engaged_) {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      cmd_pub_->publish(get_safe_cmd(latest_cmd_));
    }

    circ_chassis_msgs::msg::AEBStatus status_msg;
    status_msg.stamp              = this->now();
    status_msg.engaged            = is_engaged_;
    status_msg.ttc_s              = std::isinf(current_ttc) ? -1.0 : current_ttc;
    status_msg.min_distance_ahead_m = std::isinf(min_distance) ? -1.0 : min_distance;
    status_msg.closing_speed_mps  = v_x;
    status_msg.reason             = reason;
    status_pub_->publish(status_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<circ_chassis_msgs::msg::AEBStatus>::SharedPtr status_pub_;

  std::mutex vel_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  bool is_engaged_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBNodeC>());
  rclcpp::shutdown();
  return 0;
}