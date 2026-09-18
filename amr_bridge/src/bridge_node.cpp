// bridge_node.cpp - cmd_vel → 아두이노 패킷 변환 + 센서 패킷 → 토픽 발행
// 프로젝트1 자산(Packet) 재사용, termios 시리얼, ROS2 rclcpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include "amr_bridge/Packet.h"
#include "amr_bridge/SerialPort.h"

using namespace std::chrono_literals;

class BridgeNode : public rclcpp::Node {
public:
    BridgeNode() : Node("amr_bridge") {
        // 파라미터 (기본값: /dev/arduino, 115200)
        port_ = declare_parameter<std::string>("port", "/dev/arduino");
        baud_ = declare_parameter<int>("baud", 115200);
        lin_th_ = declare_parameter<double>("linear_threshold", 0.05);
        ang_th_ = declare_parameter<double>("angular_threshold", 0.10);

        if (!serial_.open(port_, baud_)) {
            RCLCPP_FATAL(get_logger(), "시리얼 열기 실패: %s", port_.c_str());
            throw std::runtime_error("serial open failed");
        }
        RCLCPP_INFO(get_logger(), "시리얼 연결: %s @ %d", port_.c_str(), baud_);
        rclcpp::sleep_for(2s);  // 아두이노 USB 자동 리셋 대기

        // cmd_vel 구독
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&BridgeNode::onCmdVel, this, std::placeholders::_1));

        // 센서 토픽 발행
        dist_pub_ = create_publisher<std_msgs::msg::Int32>("ultrasonic", 10);
        curr_pub_ = create_publisher<std_msgs::msg::Int32>("current", 10);

        // 하트비트 타이머 (0.3초, 워치독 방지)
        hb_timer_ = create_wall_timer(300ms, [this]() {
            serial_.write(buildArduinoCmd(CMD_HEARTBEAT));
        });

        // 시리얼 수신 타이머 (20ms마다 읽어서 센서 패킷 파싱)
        rx_timer_ = create_wall_timer(20ms, std::bind(&BridgeNode::onSerialRx, this));

        RCLCPP_INFO(get_logger(), "amr_bridge 노드 시작됨");
    }

    ~BridgeNode() {
        serial_.write(buildArduinoCmd(CMD_MOVE_STOP));  // 종료 시 정지
        serial_.close();
    }

private:
    // cmd_vel → 방향 명령 (1차: 이산 변환)
    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double lin = msg->linear.x;
        double ang = msg->angular.z;
        uint8_t cmd;

        if (lin > lin_th_)       cmd = CMD_MOVE_FWD;
        else if (lin < -lin_th_) cmd = CMD_MOVE_BACK;
        else if (ang > ang_th_)  cmd = CMD_MOVE_LEFT;
        else if (ang < -ang_th_) cmd = CMD_MOVE_RIGHT;
        else                     cmd = CMD_MOVE_STOP;

        serial_.write(buildArduinoCmd(cmd));
    }

    // 시리얼에서 센서 패킷(6바이트) 추출 → 토픽
    void onSerialRx() {
        serial_.readAvailable(rxbuf_);

        // STX~ETX 6바이트 프레임 스캔
        size_t i = 0;
        while (i + 6 <= rxbuf_.size()) {
            if (rxbuf_[i] == STX) {
                std::vector<uint8_t> frame(rxbuf_.begin() + i, rxbuf_.begin() + i + 6);
                ParseResult r = parseArduinoSensor(frame);
                if (r.ok) {
                    std_msgs::msg::Int32 m;
                    m.data = r.data[0];
                    if (r.cmd == CMD_DIST)         dist_pub_->publish(m);
                    else if (r.cmd == CMD_CURRENT) curr_pub_->publish(m);
                    i += 6;
                    continue;
                }
            }
            ++i;  // STX 아니거나 파싱 실패 → 한 칸 전진
        }
        // 처리한 부분 버림 (미완성 꼬리만 남김)
        if (i > 0) rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + i);
        // 버퍼 폭주 방지
        if (rxbuf_.size() > 512) rxbuf_.clear();
    }

    std::string port_;
    int baud_;
    double lin_th_, ang_th_;
    SerialPort serial_;
    std::vector<uint8_t> rxbuf_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dist_pub_, curr_pub_;
    rclcpp::TimerBase::SharedPtr hb_timer_, rx_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
