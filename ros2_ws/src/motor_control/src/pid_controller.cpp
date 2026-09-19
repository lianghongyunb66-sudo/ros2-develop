#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


class PIDController : public rclcpp::Node
{
public:
    PIDController()
        : Node("pid_controller")
    {
        // ==========================================
        // 1. PID 参数
        // ==========================================

        kp_ = 0.40;
        ki_ = 1.50;
        kd_ = 0.002;


        // ==========================================
        // 2. PID 控制频率
        // ==========================================

        control_frequency_ = 500.0;

        // dt = 1 / 500 = 0.002 s
        dt_ = 1.0 / control_frequency_;


        // ==========================================
        // 3. 输出和积分限制
        // ==========================================

        // 最大控制力矩
        max_torque_ = 5.0;

        // 防止积分无限累计
        max_integral_ = 3.0;


        // ==========================================
        // 4. 接收目标速度
        // ==========================================

        target_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/task3/target_speed",
                10,
                std::bind(
                    &PIDController::target_callback,
                    this,
                    std::placeholders::_1));


        // ==========================================
        // 5. 接收实际速度反馈
        // ==========================================

        omega_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/task3/omega",
                10,
                std::bind(
                    &PIDController::omega_callback,
                    this,
                    std::placeholders::_1));


        // ==========================================
        // 6. 发布PID控制力矩
        // ==========================================

        torque_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task3/torque_cmd",
                10);


        // ==========================================
        // 7. 500 Hz PID控制循环
        // ==========================================

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(2),
                std::bind(
                    &PIDController::control_loop,
                    this));


        RCLCPP_INFO(
            this->get_logger(),
            "PID controller started: control rate = 500 Hz");
    }


private:

    void target_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_speed_ = msg->data;
    }


    void omega_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_speed_ = msg->data;

        received_speed_ = true;
    }


    void control_loop()
    {
        /*
         * 如果还没收到电机速度，
         * 暂时不输出控制力矩。
         */
        if (!received_speed_)
        {
            return;
        }


        // ==========================================
        // 1. 当前速度误差
        // ==========================================

        double error =
            target_speed_
            - current_speed_;


        // ==========================================
        // 2. Integral
        // ==========================================

        integral_ +=
            error * dt_;


        // 防止积分饱和
        integral_ =
            std::clamp(
                integral_,
                -max_integral_,
                max_integral_);


        // ==========================================
        // 3. Derivative
        // ==========================================

        double derivative = 0.0;


        if (!first_update_)
        {
            derivative =
                (
                    error
                    - previous_error_
                ) / dt_;
        }


        // ==========================================
        // 4. PID控制律
        // ==========================================

        double torque =
            kp_ * error
            +
            ki_ * integral_
            +
            kd_ * derivative;


        // ==========================================
        // 5. 控制力矩限幅
        // ==========================================

        torque =
            std::clamp(
                torque,
                -max_torque_,
                max_torque_);


        // ==========================================
        // 6. 发布控制力矩
        // ==========================================

        std_msgs::msg::Float64 torque_msg;

        torque_msg.data = torque;

        torque_publisher_->publish(
            torque_msg);


        // 保存当前误差
        previous_error_ = error;

        first_update_ = false;
    }


    // PID
    double kp_;
    double ki_;
    double kd_;

    // Control timing
    double control_frequency_;
    double dt_;

    // Limits
    double max_torque_;
    double max_integral_;

    // Input data
    double target_speed_ = 0.0;
    double current_speed_ = 0.0;

    // PID states
    double integral_ = 0.0;
    double previous_error_ = 0.0;

    bool first_update_ = true;
    bool received_speed_ = false;


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_subscription_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        omega_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<PIDController>());

    rclcpp::shutdown();

    return 0;
}
