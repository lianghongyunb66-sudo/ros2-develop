#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalGenerator : public rclcpp::Node
{
public:
    SignalGenerator()
        : Node("signal_generator"),
          random_engine_(std::random_device{}()),
          gaussian_(0.0, 1.0)
    {
        // -------------------------
        // 信号参数
        // -------------------------

        amplitude_ = 1.0;          // 正弦波幅值
        signal_frequency_ = 20.0;  // 正弦波频率：20 Hz

        // 题目建议发布频率 > 500 Hz
        // 这里选择 1000 Hz
        publish_frequency_ = 1000.0;

        // 高斯噪声标准差 = 幅值的 1%
        noise_stddev_ = amplitude_ * 0.01;

        // -------------------------
        // ROS Publisher
        // -------------------------

        clean_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task1/clean",
                10);

        raw_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task1/raw",
                10);

        start_time_ = this->now();

        // 1000 Hz -> 1 ms 一次
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(
                &SignalGenerator::timer_callback,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Task1 signal generator started.");
    }

private:
    void timer_callback()
    {
        // 当前已经运行了多少秒
        double t =
            (this->now() - start_time_).seconds();

        constexpr double PI =
            3.14159265358979323846;

        // -------------------------
        // 1. 理想正弦信号
        // x(t) = A sin(2*pi*f*t)
        // -------------------------

        double clean_signal =
            amplitude_ *
            std::sin(
                2.0 *
                PI *
                signal_frequency_ *
                t);

        // -------------------------
        // 2. 高斯随机噪声
        // -------------------------

        double noise =
            noise_stddev_ *
            gaussian_(random_engine_);

        // -------------------------
        // 3. 加入噪声
        // -------------------------

        double noisy_signal =
            clean_signal + noise;

        // 发布理想信号
        std_msgs::msg::Float64 clean_msg;
        clean_msg.data = clean_signal;

        clean_publisher_->publish(clean_msg);

        // 发布带噪声信号
        std_msgs::msg::Float64 raw_msg;
        raw_msg.data = noisy_signal;

        raw_publisher_->publish(raw_msg);
    }

    double amplitude_;
    double signal_frequency_;
    double publish_frequency_;
    double noise_stddev_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        clean_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        raw_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;

    std::mt19937 random_engine_;

    std::normal_distribution<double> gaussian_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SignalGenerator>());

    rclcpp::shutdown();

    return 0;
}
