#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalFilter : public rclcpp::Node
{
public:
    SignalFilter()
        : Node("signal_filter")
    {
        // 输入数据频率
        sample_frequency_ = 1000.0;

        // 低通滤波器截止频率
        // 信号本身为 20 Hz，因此这里设为 80 Hz
        cutoff_frequency_ = 80.0;

        // 中值滤波窗口大小
        median_window_size_ = 5;

        constexpr double PI =
            3.14159265358979323846;

        // 一个采样周期
        double dt =
            1.0 / sample_frequency_;

        // RC低通模型
        double rc =
            1.0 /
            (2.0 * PI * cutoff_frequency_);

        alpha_ =
            dt / (rc + dt);

        // -------------------------
        // Publishers
        // -------------------------

        lowpass_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task1/lowpass",
                10);

        median_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task1/median",
                10);

        // -------------------------
        // Subscriber
        // -------------------------

        raw_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/task1/raw",
                10,
                std::bind(
                    &SignalFilter::raw_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Task1 filter node started.");
    }

private:
    void raw_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        const double input = msg->data;

        // =================================
        // 1. 低通滤波
        // =================================

        if (!lowpass_initialized_)
        {
            lowpass_output_ = input;
            lowpass_initialized_ = true;
        }
        else
        {
            lowpass_output_ =
                lowpass_output_ +
                alpha_ *
                (input - lowpass_output_);
        }

        std_msgs::msg::Float64 lowpass_msg;
        lowpass_msg.data = lowpass_output_;

        lowpass_publisher_->publish(lowpass_msg);

        // =================================
        // 2. 中值滤波
        // =================================

        median_buffer_.push_back(input);

        // 窗口只保留最近5个数据
        if (median_buffer_.size() >
            median_window_size_)
        {
            median_buffer_.pop_front();
        }

        std::vector<double> sorted_values(
            median_buffer_.begin(),
            median_buffer_.end());

        std::sort(
            sorted_values.begin(),
            sorted_values.end());

        double median = 0.0;

        std::size_t n =
            sorted_values.size();

        if (n % 2 == 1)
        {
            // 奇数个数据
            median =
                sorted_values[n / 2];
        }
        else
        {
            // 程序刚启动时可能暂时为偶数个数据
            median =
                (
                    sorted_values[n / 2 - 1] +
                    sorted_values[n / 2]
                ) / 2.0;
        }

        std_msgs::msg::Float64 median_msg;
        median_msg.data = median;

        median_publisher_->publish(median_msg);
    }

    double sample_frequency_;
    double cutoff_frequency_;
    double alpha_;

    std::size_t median_window_size_;

    bool lowpass_initialized_ = false;

    double lowpass_output_ = 0.0;

    std::deque<double> median_buffer_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        raw_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        lowpass_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        median_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SignalFilter>());

    rclcpp::shutdown();

    return 0;
}
