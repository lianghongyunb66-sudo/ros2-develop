#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class TorqueTestInput : public rclcpp::Node
{
public:
    TorqueTestInput()
        : Node("torque_test_input")
    {
        torque_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task2/torque_cmd",
                10);

        start_time_ = this->now();

        /*
         * 控制/输入频率：500 Hz
         *
         * 1 / 500 = 0.002 s = 2 ms
         */
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(2),
                std::bind(
                    &TorqueTestInput::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Torque test input started: command rate = 500 Hz");
    }

private:
    void timer_callback()
    {
        double elapsed_time =
            (this->now() - start_time_).seconds();

        /*
         * 每16秒循环一次
         */
        double phase =
            std::fmod(
                elapsed_time,
                16.0);

        double torque = 0.0;

        /*
         * 测试方案：
         *
         * 0 - 2 s    :  0 N.m
         * 2 - 7 s    : +1 N.m
         * 7 - 11 s   :  0 N.m
         * 11 - 16 s  : -0.5 N.m
         */

        if (phase < 2.0)
        {
            torque = 0.0;
        }
        else if (phase < 7.0)
        {
            torque = 1.0;
        }
        else if (phase < 11.0)
        {
            torque = 0.0;
        }
        else
        {
            torque = -0.5;
        }

        std_msgs::msg::Float64 msg;

        msg.data = torque;

        torque_publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<TorqueTestInput>());

    rclcpp::shutdown();

    return 0;
}
