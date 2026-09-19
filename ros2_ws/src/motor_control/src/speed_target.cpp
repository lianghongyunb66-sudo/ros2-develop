#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


class SpeedTarget : public rclcpp::Node
{
public:
    SpeedTarget()
        : Node("speed_target")
    {
        target_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task3/target_speed",
                10);

        start_time_ = this->now();


        /*
         * 目标速度不需要500Hz不断改变，
         * 这里使用50Hz发布目标值。
         *
         * 1 / 50 = 0.02s = 20ms
         */
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &SpeedTarget::timer_callback,
                    this));


        RCLCPP_INFO(
            this->get_logger(),
            "Speed target started.");
    }


private:

    void timer_callback()
    {
        double elapsed_time =
            (this->now() - start_time_).seconds();


        /*
         * 每20秒循环一次
         */
        double phase =
            std::fmod(
                elapsed_time,
                20.0);


        double target_speed = 0.0;


        /*
         * 目标速度测试方案：
         *
         * 0  ~ 2 秒      0 rad/s
         * 2  ~ 8 秒     20 rad/s
         * 8  ~ 14 秒    10 rad/s
         * 14 ~ 20 秒     0 rad/s
         */

        if (phase < 2.0)
        {
            target_speed = 0.0;
        }
        else if (phase < 8.0)
        {
            target_speed = 20.0;
        }
        else if (phase < 14.0)
        {
            target_speed = 10.0;
        }
        else
        {
            target_speed = 0.0;
        }


        std_msgs::msg::Float64 msg;
        msg.data = target_speed;

        target_publisher_->publish(msg);
    }


    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SpeedTarget>());

    rclcpp::shutdown();

    return 0;
}
