
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class ChassisBridge : public rclcpp::Node
{
public:
    ChassisBridge() : Node("chassis_bridge_node")
    {
        // 串口参数
        port_ = declare_parameter<std::string>(
            "port", "/dev/ttyUSB1");

        baud_ = declare_parameter<int>(
            "baud", 115200);

        openSerial();

        // TF 广播器
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(
                *this);

        // 订阅 /cmd_vel
        cmd_sub_ = create_subscription<
            geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &ChassisBridge::cmdCallback,
                this,
                std::placeholders::_1));

        // 发布 /chassis/pose
        pose_pub_ = create_publisher<
            geometry_msgs::msg::PoseStamped>(
            "/chassis/pose", 10);

        // 发布 /odom
        odom_pub_ = create_publisher<
            nav_msgs::msg::Odometry>(
            "/odom", 10);

        // 20 Hz 发送速度
        send_timer_ = create_wall_timer(
            50ms,
            std::bind(
                &ChassisBridge::sendTimer,
                this));

        // 每 5 ms 轮询一次串口接收
        read_timer_ = create_wall_timer(
            5ms,
            std::bind(
                &ChassisBridge::readSerial,
                this));

        RCLCPP_INFO(
            get_logger(),
            "Chassis bridge started");
    }

    ~ChassisBridge()
    {
        if (fd_ >= 0) {
            sendCommand(0.0, 0.0, 0.0);
            close(fd_);
        }
    }

private:
    // ========== 串口参数 ==========
    int fd_ = -1;
    int baud_ = 115200;

    std::string port_;

    // ========== 速度指令 ==========
    double vx_ = 0.0;
    double vy_ = 0.0;
    double wz_ = 0.0;

    bool received_cmd_ = false;

    std::chrono::steady_clock::time_point
        last_cmd_time_;

    // ========== 串口接收缓冲区 ==========
    std::vector<uint8_t> rx_buffer_;

    bool has_sequence_ = false;
    uint16_t last_sequence_ = 0;

    // ========== 上一帧里程计数据 ==========
    bool has_last_odom_ = false;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_yaw_ = 0.0;

    uint32_t last_timestamp_ms_ = 0;

    // ========== ROS 2 通信对象 ==========
    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<
        tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    // ========== 串口初始化 ==========
    void openSerial()
    {
        fd_ = open(
            port_.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd_ < 0) {
            throw std::runtime_error(
                "Cannot open serial: " + port_);
        }

        termios tty{};

        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "tcgetattr failed");
        }

        cfmakeraw(&tty);

        // 8N1
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8 | CLOCAL | CREAD;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        speed_t speed;

        if (baud_ == 115200) {
            speed = B115200;
        } else if (baud_ == 9600) {
            speed = B9600;
        } else {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "Unsupported baud rate");
        }

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "tcsetattr failed");
        }

        RCLCPP_INFO(
            get_logger(),
            "Serial opened: %s, baud=%d",
            port_.c_str(), baud_);
    }

    // ========== 接收 ROS 2 速度 ==========
    void cmdCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!std::isfinite(msg->linear.x) ||
            !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z)) {
            return;
        }

        // 保留原有速度限制
        vx_ = std::clamp(msg->linear.x, -1.0, 1.0);
        vy_ = std::clamp(msg->linear.y, -1.0, 1.0);
        wz_ = std::clamp(msg->angular.z, -1.0, 1.0);

        received_cmd_ = true;

        last_cmd_time_ =
            std::chrono::steady_clock::now();
    }

    // ========== 定时发送速度 ==========
    void sendTimer()
    {
        auto now = std::chrono::steady_clock::now();

        // 300 ms 未收到新指令，发送零速度
        if (!received_cmd_ ||
            now - last_cmd_time_ > 300ms) {

            sendCommand(0.0, 0.0, 0.0);
            return;
        }

        sendCommand(vx_, vy_, wz_);
    }

    // ========== ASCII 控制协议 ==========
    void sendCommand(double x, double y, double z)
    {
        if (fd_ < 0) {
            return;
        }

        char buffer[80];

        int length = std::snprintf(
            buffer,
            sizeof(buffer),
            "[v,%.3f,%.3f,%.3f]",
            x, y, z);

        if (length <= 0 ||
            length >= static_cast<int>(sizeof(buffer))) {
            return;
        }

        size_t sent = 0;
        size_t total = static_cast<size_t>(length);

        while (sent < total) {
            ssize_t n = write(
                fd_,
                buffer + sent,
                total - sent);

            if (n > 0) {
                sent += static_cast<size_t>(n);
                continue;
            }

            if (n < 0 && errno == EINTR) {
                continue;
            }

            if (n < 0 &&
                (errno == EAGAIN ||
                 errno == EWOULDBLOCK)) {

                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Serial TX buffer busy");
                break;
            }

            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Serial write failed");
            break;
        }

        if (sent != total) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Incomplete chassis command");
        }
    }

    // ========== CRC16/MODBUS ==========
    static uint16_t crc16Modbus(
        const uint8_t *data,
        size_t length)
    {
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];

            for (int j = 0; j < 8; ++j) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

    // ========== 小端数据解析 ==========
    static uint16_t readU16(const uint8_t *p)
    {
        return static_cast<uint16_t>(p[0]) |
               (static_cast<uint16_t>(p[1]) << 8);
    }

    static uint32_t readU32(const uint8_t *p)
    {
        return static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
    }

    static float readFloat32(const uint8_t *p)
    {
        static_assert(
            sizeof(float) == 4,
            "float must be 32 bits");

        uint32_t bits = readU32(p);

        float value;
        std::memcpy(&value, &bits, sizeof(value));

        return value;
    }

    // ========== 读取 STM32 坐标数据 ==========
    void readSerial()
    {
        if (fd_ < 0) {
            return;
        }

        uint8_t buffer[256];

        // 每次最多读取 8 轮
        for (int i = 0; i < 8; ++i) {
            ssize_t n = read(
                fd_, buffer, sizeof(buffer));

            if (n > 0) {
                rx_buffer_.insert(
                    rx_buffer_.end(),
                    buffer,
                    buffer + n);

                continue;
            }

            if (n < 0 && errno == EINTR) {
                continue;
            }

            if (n < 0 &&
                errno != EAGAIN &&
                errno != EWOULDBLOCK) {

                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Serial read failed");
            }

            break;
        }

        parseFrames();
    }

    // ========== 解析 24 字节坐标帧 ==========
    void parseFrames()
    {
        constexpr size_t FRAME_SIZE = 24;

        while (rx_buffer_.size() >= 4) {

            // 帧头 AA 55
            if (rx_buffer_[0] != 0xAA ||
                rx_buffer_[1] != 0x55) {

                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // 类型 01，长度 24
            if (rx_buffer_[2] != 0x01 ||
                rx_buffer_[3] != FRAME_SIZE) {

                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            if (rx_buffer_.size() < FRAME_SIZE) {
                break;
            }

            const uint8_t *frame = rx_buffer_.data();

            // CRC 校验
            uint16_t calculated_crc =
                crc16Modbus(frame, 22);

            uint16_t received_crc =
                readU16(frame + 22);

            if (calculated_crc != received_crc) {

                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Position frame CRC error");

                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // 数据解析
            uint16_t sequence =
                readU16(frame + 4);

            uint32_t timestamp_ms =
                readU32(frame + 6);

            float x = readFloat32(frame + 10);
            float y = readFloat32(frame + 14);
            float o_deg = readFloat32(frame + 18);

            if (std::isfinite(x) &&
                std::isfinite(y) &&
                std::isfinite(o_deg)) {

                // 序号检查
                if (has_sequence_) {
                    uint16_t delta =
                        static_cast<uint16_t>(
                            sequence - last_sequence_);

                    // 丢弃重复帧
                    if (delta == 0) {
                        rx_buffer_.erase(
                            rx_buffer_.begin(),
                            rx_buffer_.begin() + FRAME_SIZE);
                        continue;
                    }

                    // 发现序号跳变
                    if (delta != 1) {
                        RCLCPP_WARN_THROTTLE(
                            get_logger(),
                            *get_clock(),
                            2000,
                            "Position sequence discontinuity");

                        // 先不使用跨缺帧数据计算速度
                        has_last_odom_ = false;
                    }
                }

                last_sequence_ = sequence;
                has_sequence_ = true;

                publishPose(
                    x, y, o_deg,
                    sequence, timestamp_ms);
            }

            rx_buffer_.erase(
                rx_buffer_.begin(),
                rx_buffer_.begin() + FRAME_SIZE);
        }

        if (rx_buffer_.size() > 4096) {
            rx_buffer_.clear();
        }
    }

    // ========== 发布坐标 / TF / 里程计 ==========
    void publishPose(
        float x,
        float y,
        float o_deg,
        uint16_t sequence,
        uint32_t timestamp_ms)
    {
        constexpr double PI =
            3.14159265358979323846;

        double yaw =
            static_cast<double>(o_deg) * PI / 180.0;

        geometry_msgs::msg::PoseStamped pose;

        // 同一帧数据使用相同 ROS 时间戳
        pose.header.stamp = this->now();
        pose.header.frame_id = "odom";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        // 二维航向角转四元数
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);

        // 1. 位姿
        pose_pub_->publish(pose);

        // 2. 标准里程计
        publishOdometry(pose, yaw, timestamp_ms);

        // 3. 动态 TF
        publishTF(pose);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Pose: x=%.3f y=%.3f yaw=%.2f deg "
            "seq=%u time=%u ms",
            x, y, o_deg,
            static_cast<unsigned>(sequence),
            static_cast<unsigned>(timestamp_ms));
    }

    // ========== 发布标准 /odom ==========
    void publishOdometry(
        const geometry_msgs::msg::PoseStamped &pose,
        double yaw,
        uint32_t timestamp_ms)
    {
        nav_msgs::msg::Odometry odom;

        odom.header = pose.header;
        odom.child_frame_id = "base_link";

        // 位姿来自 STM32
        odom.pose.pose = pose.pose;

        double x = pose.pose.position.x;
        double y = pose.pose.position.y;

        double vx_body = 0.0;
        double vy_body = 0.0;
        double wz = 0.0;

        bool velocity_valid = false;

        if (has_last_odom_) {

            // 利用 uint32_t 无符号减法处理计时回绕
            uint32_t dt_ms =
                timestamp_ms - last_timestamp_ms_;

            // 过滤重复及异常时间间隔
            if (dt_ms >= 5 && dt_ms <= 200) {

                double dt =
                    static_cast<double>(dt_ms) / 1000.0;

                // 世界坐标系中的位置差
                double dx = x - last_x_;
                double dy = y - last_y_;

                double vx_odom = dx / dt;
                double vy_odom = dy / dt;

                // 世界速度 -> 车体速度
                double c = std::cos(yaw);
                double s = std::sin(yaw);

                vx_body =
                    c * vx_odom + s * vy_odom;

                vy_body =
                    -s * vx_odom + c * vy_odom;

                // 计算航向角差
                double delta_yaw =
                    yaw - last_yaw_;

                delta_yaw = std::atan2(
                    std::sin(delta_yaw),
                    std::cos(delta_yaw));

                wz = delta_yaw / dt;

                velocity_valid = true;
            }
        }

        // 保存本次坐标供下一帧使用
        last_x_ = x;
        last_y_ = y;
        last_yaw_ = yaw;
        last_timestamp_ms_ = timestamp_ms;

        has_last_odom_ = true;

        // ========== 实际速度 ==========
        odom.twist.twist.linear.x = vx_body;
        odom.twist.twist.linear.y = vy_body;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = wz;

        // ========== 协方差 ==========
        // 以下为初期调试占位值，需要实测标定
        odom.pose.covariance.fill(0.0);
        odom.twist.covariance.fill(0.0);

        // X、Y、Yaw
        odom.pose.covariance[0] = 0.0025;
        odom.pose.covariance[7] = 0.0025;
        odom.pose.covariance[35] = 0.03;

        // 未观测的 Z、Roll、Pitch
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;

        // 速度方差
        double linear_variance =
            velocity_valid ? 0.01 : 1e6;

        double angular_variance =
            velocity_valid ? 0.04 : 1e6;

        odom.twist.covariance[0] =
            linear_variance;

        odom.twist.covariance[7] =
            linear_variance;

        odom.twist.covariance[35] =
            angular_variance;

        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;

        // 发布标准里程计
        odom_pub_->publish(odom);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Odom: vx=%.3f vy=%.3f wz=%.3f",
            vx_body, vy_body, wz);
    }

    // ========== 发布动态 TF ==========
    void publishTF(
        const geometry_msgs::msg::PoseStamped &pose)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header = pose.header;
        tf_msg.child_frame_id = "base_link";

        // 平移
        tf_msg.transform.translation.x =
            pose.pose.position.x;

        tf_msg.transform.translation.y =
            pose.pose.position.y;

        tf_msg.transform.translation.z =
            pose.pose.position.z;

        // 旋转
        tf_msg.transform.rotation =
            pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int result = 0;

    try {
        rclcpp::spin(
            std::make_shared<ChassisBridge>());
    }
    catch (const std::exception &e) {
        std::fprintf(
            stderr,
            "Chassis bridge error: %s\n",
            e.what());

        result = 1;
    }

    rclcpp::shutdown();

    return result;
}
