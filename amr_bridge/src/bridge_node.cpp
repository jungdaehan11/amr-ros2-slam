// bridge_node.cpp - cmd_vel → 아두이노 패킷 변환 + 센서 패킷 → 토픽 발행
// ★4단계: 연속 PWM(0x15) 변환 + 기동 부스트(kick-start).
//   정지 상태에서 새로 움직이기 시작할 때 정지마찰(특히 제자리 회전 scrub)을
//   깨기 위해 초기 몇 틱만 높은 PWM으로 밀고, 이후 계산값으로 복귀.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <cmath>
#include "amr_bridge/Packet.h"
#include "amr_bridge/SerialPort.h"

using namespace std::chrono_literals;

class BridgeNode : public rclcpp::Node {
public:
    BridgeNode() : Node("amr_bridge") {
        port_ = declare_parameter<std::string>("port", "/dev/arduino");
        baud_ = declare_parameter<int>("baud", 115200);

        inv_a_      = declare_parameter<double>("inv_a", 409.55);
        inv_b_      = declare_parameter<double>("inv_b", -12.91);
        onset_int8_ = declare_parameter<int>("onset_int8", 80);
        max_int8_   = declare_parameter<int>("max_int8", 127);
        wheel_sep_  = declare_parameter<double>("wheel_separation", 0.138);
        right_trim_ = declare_parameter<int>("right_trim", 4);
        stop_eps_   = declare_parameter<double>("stop_epsilon", 0.01);
        cmd_timeout_= declare_parameter<double>("cmd_timeout", 0.5);
        turn_gain_  = declare_parameter<double>("turn_gain", 3.0);

        boost_add_   = declare_parameter<int>("boost_add", 40);
        boost_ticks_ = declare_parameter<int>("boost_ticks", 4);

        // ★IMU 파라미터
        gyro_scale_ = declare_parameter<double>("gyro_scale", 131.0);   // LSB per deg/s (±250dps)
        gyro_sign_  = declare_parameter<double>("gyro_sign", 1.0);      // 부호 뒤집기: -1.0
        imu_frame_  = declare_parameter<std::string>("imu_frame_id", "imu_link");
        imu_var_    = declare_parameter<double>("imu_angular_variance", 0.0004);

        if (!serial_.open(port_, baud_)) {
            RCLCPP_FATAL(get_logger(), "시리얼 열기 실패: %s", port_.c_str());
            throw std::runtime_error("serial open failed");
        }
        RCLCPP_INFO(get_logger(), "시리얼 연결: %s @ %d", port_.c_str(), baud_);
        rclcpp::sleep_for(2s);

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&BridgeNode::onCmdVel, this, std::placeholders::_1));
        dist_pub_ = create_publisher<std_msgs::msg::Int32>("ultrasonic", 10);
        curr_pub_ = create_publisher<std_msgs::msg::Int32>("current", 10);
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("imu", 50);

        last_cmd_time_ = now();
        tx_timer_ = create_wall_timer(50ms, std::bind(&BridgeNode::onTxTimer, this));
        rx_timer_ = create_wall_timer(5ms, std::bind(&BridgeNode::onSerialRx, this));

        RCLCPP_INFO(get_logger(), "amr_bridge 시작됨 (0x15 연속PWM + 기동부스트)");
    }

    ~BridgeNode() {
        serial_.write(buildArduinoSetPWM(0, 0));
        serial_.close();
    }

private:
    int8_t velToInt8(double v) {
        if (std::fabs(v) < stop_eps_) return 0;
        double mag = inv_a_ * std::fabs(v) + inv_b_;
        if (mag < onset_int8_) mag = onset_int8_;
        if (mag > max_int8_)   mag = max_int8_;
        int val = static_cast<int>(std::lround(mag));
        return static_cast<int8_t>(v >= 0 ? val : -val);
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double lin = msg->linear.x;
        double ang = msg->angular.z;
        double v_left  = lin - ang * (wheel_sep_ / 2.0) * turn_gain_;
        double v_right = lin + ang * (wheel_sep_ / 2.0) * turn_gain_;
        int8_t l = velToInt8(v_left);
        int8_t r = velToInt8(v_right);
        if (r > 0)      r = static_cast<int8_t>(std::max(0, r - right_trim_));
        else if (r < 0) r = static_cast<int8_t>(std::min(0, r + right_trim_));

        bool was_stopped = (target_l_ == 0 && target_r_ == 0);
        bool now_moving  = (l != 0 || r != 0);
        if (was_stopped && now_moving && boost_ticks_ > 0) {
            boost_remaining_ = boost_ticks_;
        }

        target_l_ = l;
        target_r_ = r;
        last_cmd_time_ = now();
    }

    int8_t applyBoost(int8_t v) {
        if (v == 0) return 0;
        int mag = std::abs((int)v) + boost_add_;
        if (mag > max_int8_) mag = max_int8_;
        return static_cast<int8_t>(v > 0 ? mag : -mag);
    }

    void onTxTimer() {
        if ((now() - last_cmd_time_).seconds() > cmd_timeout_) {
            target_l_ = 0; target_r_ = 0;
            boost_remaining_ = 0;
        }

        int8_t out_l = target_l_, out_r = target_r_;
        if (boost_remaining_ > 0) {
            out_l = applyBoost(target_l_);
            out_r = applyBoost(target_r_);
            --boost_remaining_;
        }
        serial_.write(buildArduinoSetPWM(out_l, out_r));
    }

    void publishImu(int16_t raw) {
        sensor_msgs::msg::Imu m;
        m.header.stamp = now();
        m.header.frame_id = imu_frame_;

        // raw → deg/s → rad/s, 부호 파라미터 적용
        double dps = static_cast<double>(raw) / gyro_scale_;
        m.angular_velocity.z = gyro_sign_ * dps * M_PI / 180.0;

        // 이 IMU는 yaw 각속도만 제공: 나머지는 "없음" 표시
        m.orientation_covariance[0] = -1.0;          // orientation 미제공
        m.linear_acceleration_covariance[0] = -1.0;  // 가속도 미제공
        // ★주의: 공분산[0]=-1은 ROS 규약상 "각속도 전체 없음"을 뜻하므로 쓰면 안 됨.
        //   안 쓰는 축은 아주 큰 분산(=신뢰도 0)으로 표현한다.
        m.angular_velocity_covariance[0] = 1e6;      // x 사실상 무시
        m.angular_velocity_covariance[4] = 1e6;      // y 사실상 무시
        m.angular_velocity_covariance[8] = imu_var_; // z만 유효

        imu_pub_->publish(m);
    }

    // ★가변길이 수신: LEN으로 6바이트(센서) / 7바이트(IMU) 분기
    //   총 길이 = LEN + 5  (STX LEN CMD ...DATA... CHK ETX)
    void onSerialRx() {
        serial_.readAvailable(rxbuf_);
        size_t i = 0;
        while (i < rxbuf_.size()) {
            if (rxbuf_[i] != STX) { ++i; continue; }
            if (i + 1 >= rxbuf_.size()) break;        // LEN 아직 안 옴

            uint8_t len = rxbuf_[i + 1];
            if (len != 0x01 && len != 0x02) { ++i; continue; }   // 미지원 길이

            size_t total = static_cast<size_t>(len) + 5;
            if (i + total > rxbuf_.size()) break;     // 프레임 미완성 → 다음 틱

            std::vector<uint8_t> frame(rxbuf_.begin() + i, rxbuf_.begin() + i + total);

            if (len == 0x01) {
                ParseResult r = parseArduinoSensor(frame);
                if (r.ok) {
                    std_msgs::msg::Int32 m;
                    m.data = r.data[0];
                    if (r.cmd == CMD_DIST)         dist_pub_->publish(m);
                    else if (r.cmd == CMD_CURRENT) curr_pub_->publish(m);
                    i += total;
                    continue;
                }
            } else {
                ParseResult r = parseArduinoSensor16(frame);
                if (r.ok && r.cmd == CMD_GYRO_Z) {
                    publishImu(dataToInt16(r.data));
                    i += total;
                    continue;
                }
            }
            ++i;   // 파싱 실패 → 1바이트 밀고 재동기화
        }
        if (i > 0) rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + i);
        if (rxbuf_.size() > 512) rxbuf_.clear();
    }

    std::string port_;
    int baud_;
    double inv_a_, inv_b_;
    int onset_int8_, max_int8_;
    double wheel_sep_;
    int right_trim_;
    double stop_eps_, cmd_timeout_;
    double turn_gain_;
    int boost_add_, boost_ticks_;
    double gyro_scale_, gyro_sign_, imu_var_;
    std::string imu_frame_;

    SerialPort serial_;
    std::vector<uint8_t> rxbuf_;
    int8_t target_l_ = 0, target_r_ = 0;
    int boost_remaining_ = 0;
    rclcpp::Time last_cmd_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dist_pub_, curr_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr tx_timer_, rx_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
