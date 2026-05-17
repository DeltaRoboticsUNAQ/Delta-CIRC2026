#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <circ_chassis_msgs/msg/aeb_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <deque>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <limits>

class UnifiedSafetyController : public rclcpp::Node
{
public:
    UnifiedSafetyController() : Node("unified_safety_controller"),
                                 min_distance_(std::numeric_limits<double>::infinity()),
                                 current_pitch_(0.0),
                                 current_roll_(0.0),
                                 z_accel_variance_(0.0),
                                 aeb_engaged_(false)
    {
        // ── Parámetros generales ──────────────────────────────────────────
        this->declare_parameter("scan_topic",    "/scan");
        this->declare_parameter("imu_topic",     "/imu/data_raw");
        this->declare_parameter("cmd_topic_in",  "/cmd_vel_raw");
        this->declare_parameter("cmd_topic_out", "/cmd_vel");
        this->declare_parameter("status_topic",  "/chassis/safety/aeb_status");

        // ── Parámetros LiDAR / corredor frontal ──────────────────────────
        this->declare_parameter("front_angle_span_deg", 60.0);
        this->declare_parameter("robot_width_m",         0.6);

        // ── Parámetros Adaptive Speed ─────────────────────────────────────
        this->declare_parameter("warning_distance",       2.0);   // m  → empieza a reducir
        this->declare_parameter("critical_distance",      0.6);   // m  → S_obs = 0
        this->declare_parameter("max_incline_deg",       18.0);   // deg → S_inc = 0
        this->declare_parameter("vibration_buffer_size",  50);    // muestras IMU
        this->declare_parameter("max_vibration_variance", 4.0);   // (m/s²)²
        this->declare_parameter("z_gravity_offset",       9.81);  // m/s²

        // ── Parámetros AEB ────────────────────────────────────────────────
        this->declare_parameter("ttc_threshold_s",          1.2); // s
        this->declare_parameter("min_distance_threshold_m", 0.6); // m
        this->declare_parameter("braking_distance_m",       0.4); // m

        // ── Suscripciones ─────────────────────────────────────────────────
        auto sensor_qos = rclcpp::SensorDataQoS();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), sensor_qos,
            std::bind(&UnifiedSafetyController::scan_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), sensor_qos,
            std::bind(&UnifiedSafetyController::imu_callback, this, std::placeholders::_1));

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_topic_in").as_string(), 10,
            std::bind(&UnifiedSafetyController::cmd_callback, this, std::placeholders::_1));

        // ── Publicadores ──────────────────────────────────────────────────
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_topic_out").as_string(), 10);

        status_pub_ = this->create_publisher<circ_chassis_msgs::msg::AEBStatus>(
            this->get_parameter("status_topic").as_string(), 10);

        RCLCPP_INFO(this->get_logger(), "Unified Safety Controller iniciado.");
        RCLCPP_INFO(this->get_logger(), "Pipeline: %s -> [AdaptiveSpeed + AEB] -> %s",
            this->get_parameter("cmd_topic_in").as_string().c_str(),
            this->get_parameter("cmd_topic_out").as_string().c_str());
    }

private:
    // ── Suscripciones / Publicadores ─────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
    rclcpp::Publisher<circ_chassis_msgs::msg::AEBStatus>::SharedPtr status_pub_;

    // ── Estado compartido entre callbacks ────────────────────────────────
    std::mutex sensor_mutex_;
    double min_distance_;       // distancia mínima en corredor frontal (m)
    double current_pitch_;      // rad
    double current_roll_;       // rad
    double z_accel_variance_;   // varianza aceleración Z centrada
    std::deque<double> z_accel_buffer_;

    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist latest_cmd_;
    bool aeb_engaged_;

    // ═════════════════════════════════════════════════════════════════════
    // CALLBACK: LiDAR  →  actualiza min_distance_ (corredor frontal)
    // ═════════════════════════════════════════════════════════════════════
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double span_rad    = this->get_parameter("front_angle_span_deg").as_double() * M_PI / 180.0;
        double robot_width = this->get_parameter("robot_width_m").as_double();

        double current_min = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            if (std::abs(angle) >= (span_rad / 2.0)) continue;

            double y_pos = std::abs(r * std::sin(angle));
            if (y_pos >= (robot_width / 2.0 + 0.1)) continue;

            double x_proj = r * std::cos(angle);
            if (x_proj < current_min) current_min = x_proj;
        }

        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            min_distance_ = current_min;
        }

        // Evaluar la seguridad inmediatamente cuando llega nuevo scan,
        // para frenar aunque no estén llegando comandos nuevos.
        evaluate_and_publish_safety(true);
    }

    // ═════════════════════════════════════════════════════════════════════
    // CALLBACK: IMU  →  actualiza pitch, roll y varianza de vibración
    // ═════════════════════════════════════════════════════════════════════
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Euler desde cuaternión
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Aceleración Z centrada (quita la gravedad estática)
        double z_gravity  = this->get_parameter("z_gravity_offset").as_double();
        double z_centered = msg->linear_acceleration.z - z_gravity;

        int buf_size = this->get_parameter("vibration_buffer_size").as_int();

        // Construir buffer localmente y calcular varianza fuera del lock
        std::deque<double> local_buf;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            z_accel_buffer_.push_back(z_centered);
            if ((int)z_accel_buffer_.size() > buf_size) z_accel_buffer_.pop_front();
            local_buf = z_accel_buffer_;
        }

        double variance = 0.0;
        if (local_buf.size() > 1)
        {
            double sum  = std::accumulate(local_buf.begin(), local_buf.end(), 0.0);
            double mean = sum / local_buf.size();
            double sq_sum = std::inner_product(
                local_buf.begin(), local_buf.end(), local_buf.begin(), 0.0,
                std::plus<double>(),
                [mean](double a, double b){ return (a - mean) * (b - mean); });
            variance = sq_sum / (local_buf.size() - 1);
        }

        std::lock_guard<std::mutex> lock(sensor_mutex_);
        current_roll_     = roll;
        current_pitch_    = pitch;
        z_accel_variance_ = variance;
    }

    // ═════════════════════════════════════════════════════════════════════
    // CALLBACK: cmd_vel_raw  →  aplica Adaptive Speed + AEB en cascada
    // ═════════════════════════════════════════════════════════════════════
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Guardar último comando
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            latest_cmd_ = *msg;
        }

        evaluate_and_publish_safety(false);
    }

    void evaluate_and_publish_safety(bool from_scan)
    {
        // Leer el último comando
        geometry_msgs::msg::Twist raw_cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            raw_cmd = latest_cmd_;
        }

        // Leer estado de sensores de forma segura
        double min_dist, pitch, roll, variance;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            min_dist = min_distance_;
            pitch    = current_pitch_;
            roll     = current_roll_;
            variance = z_accel_variance_;
        }

        // ── CAPA 1: Adaptive Speed ────────────────────────────────────────
        geometry_msgs::msg::Twist adapted = apply_adaptive_speed(raw_cmd, min_dist, pitch, roll, variance);

        // ── CAPA 2: AEB Hard Stop ─────────────────────────────────────────
        geometry_msgs::msg::Twist safe    = apply_aeb(adapted, raw_cmd, min_dist);

        // Si evaluamos desde el scan, solo publicamos si estamos interviniendo (AEB) 
        // o si el comando de avance es distinto de cero. Así evitamos enviar ceros infinitamente.
        if (from_scan) {
            if (aeb_engaged_ || raw_cmd.linear.x != 0.0 || raw_cmd.angular.z != 0.0) {
                cmd_pub_->publish(safe);
            }
        } else {
            // Desde cmd_callback publicamos normal
            cmd_pub_->publish(safe);
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    // CAPA 1: Adaptive Speed — multiplica velocidad según 3 factores
    // ═════════════════════════════════════════════════════════════════════
    geometry_msgs::msg::Twist apply_adaptive_speed(
        const geometry_msgs::msg::Twist & cmd,
        double min_dist, double pitch, double roll, double variance)
    {
        double warn_dist   = this->get_parameter("warning_distance").as_double();
        double crit_dist   = this->get_parameter("critical_distance").as_double();
        double max_inc_rad = this->get_parameter("max_incline_deg").as_double() * M_PI / 180.0;
        double max_vib     = this->get_parameter("max_vibration_variance").as_double();

        // Factor 1: Proximidad (solo si avanza)
        double S_obs = 1.0;
        if (cmd.linear.x > 0.0) {
            if (min_dist <= crit_dist) {
                S_obs = 0.0;
            } else if (min_dist < warn_dist) {
                S_obs = (min_dist - crit_dist) / (warn_dist - crit_dist);
            }
        }

        // Factor 2: Inclinación
        double S_inc = 1.0;
        double max_inc = std::max(std::abs(pitch), std::abs(roll));
        if (max_inc > max_inc_rad) {
            S_inc = 0.0;
        } else if (max_inc > (max_inc_rad * 0.5)) {
            S_inc = 1.0 - ((max_inc - (max_inc_rad * 0.5)) / (max_inc_rad * 0.5));
        }

        // Factor 3: Vibración
        double S_vib = 1.0;
        if (variance > max_vib) {
            S_vib = 0.2;
        } else if (variance > (max_vib * 0.3)) {
            S_vib = 1.0 - (0.8 * ((variance - (max_vib * 0.3)) / (max_vib * 0.7)));
        }

        double multiplier = std::clamp(S_obs * S_inc * S_vib, 0.0, 1.0);

        geometry_msgs::msg::Twist out = cmd;

        // Solo modular avance frontal; reversa pasa libre
        if (cmd.linear.x > 0.0) out.linear.x = cmd.linear.x * multiplier;
        out.linear.y  *= multiplier;
        out.angular.z *= multiplier;

        // Log de modulación activa
        if (multiplier < 0.9 && std::abs(cmd.linear.x) > 0.01) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[AdaptiveSpeed] Modulando: (Dist: %.2fm | Inc: %.1fdeg | Vib: %.3f) -> Mul: %.2f",
                min_dist, max_inc * (180.0 / M_PI), variance, multiplier);
        }

        // Log de estado general cada 2 segundos
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[Estado] S_obs=%.2f | S_inc=%.2f | S_vib=%.2f | Mul=%.2f | vx_in=%.2f -> vx_out=%.2f",
            S_obs, S_inc, S_vib, multiplier, cmd.linear.x, out.linear.x);

        return out;
    }

    // ═════════════════════════════════════════════════════════════════════
    // CAPA 2: AEB — freno de emergencia duro si TTC o distancia crítica
    // ═════════════════════════════════════════════════════════════════════
    geometry_msgs::msg::Twist apply_aeb(
        const geometry_msgs::msg::Twist & adapted_cmd,
        const geometry_msgs::msg::Twist & raw_cmd,
        double min_dist)
    {
        double ttc_threshold   = this->get_parameter("ttc_threshold_s").as_double();
        double min_dist_thresh = this->get_parameter("min_distance_threshold_m").as_double();
        double braking_dist    = this->get_parameter("braking_distance_m").as_double();

        // Usamos el comando RAW (intención del usuario) para predecir si chocaría
        // (porque el adapted_cmd ya puede estar en 0 por el Adaptive Speed)
        double v_x = raw_cmd.linear.x;

        bool   should_engage = false;
        double current_ttc   = std::numeric_limits<double>::infinity();
        std::string reason   = "Safe";

        // AEB solo actúa si hay obstáculo detectado y el rover avanza
        if (std::isfinite(min_dist) && v_x > 0.0)
        {
            double effective_threshold = min_dist_thresh + braking_dist;

            if (min_dist <= effective_threshold) {
                should_engage = true;
                reason = "Distancia critica (" + std::to_string(min_dist) + "m)";
            } else if (v_x > 0.05) {
                current_ttc = min_dist / v_x;
                double effective_ttc = ttc_threshold + (braking_dist / v_x);
                if (current_ttc <= effective_ttc) {
                    should_engage = true;
                    reason = "TTC violation (" + std::to_string(current_ttc) + "s)";
                }
            }
        }

        // Logs de cambio de estado AEB
        if (should_engage && !aeb_engaged_) {
            RCLCPP_WARN(this->get_logger(), "⚠️  AEB ENGAGED! Razon: %s", reason.c_str());
        } else if (!should_engage && aeb_engaged_) {
            RCLCPP_INFO(this->get_logger(), "✅ AEB DISENGAGED. Camino libre.");
        }

        aeb_engaged_ = should_engage;

        // Publicar telemetría AEB
        circ_chassis_msgs::msg::AEBStatus status;
        status.stamp                = this->now();
        status.engaged              = aeb_engaged_;
        status.ttc_s                = std::isinf(current_ttc) ? -1.0 : current_ttc;
        status.min_distance_ahead_m = std::isinf(min_dist)    ? -1.0 : min_dist;
        status.closing_speed_mps    = v_x;
        status.reason               = reason;
        status_pub_->publish(status);

        // Si AEB activo: bloquear solo avance, reversa y giro libres
        geometry_msgs::msg::Twist safe = adapted_cmd;
        if (aeb_engaged_) {
            if (safe.linear.x > 0.0) safe.linear.x = 0.0;
        }

        return safe;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedSafetyController>());
    rclcpp::shutdown();
    return 0;
}