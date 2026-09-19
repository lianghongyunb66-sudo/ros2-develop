#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


class MotorSimulator : public rclcpp::Node
{
public:
    MotorSimulator()
        : Node("motor_simulator")
    {
        // =========================================
        // 1. 电机机械模型参数
        // =========================================

        // 转动惯量 J
        J_ = 0.02;

        // 粘性阻尼系数 B
        B_ = 0.10;

        // 外部负载力矩 TL
        // 本次基础实验先设为 0
        load_torque_ = 0.0;


        // =========================================
        // 2. 电机状态计算频率
        // =========================================

        // 每秒计算 1000 次
        simulation_frequency_ = 1000.0;

        // dt = 1 / 1000 = 0.001 s
        dt_ = 1.0 / simulation_frequency_;


        // =========================================
        // 3. 接收控制力矩
        // =========================================

        torque_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/task2/torque_cmd",
                10,
                std::bind(
                    &MotorSimulator::torque_callback,
                    this,
                    std::placeholders::_1));


        // =========================================
        // 4. 发布电机角速度
        // =========================================

        omega_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task2/omega",
                10);


        // =========================================
        // 5. 发布电机角度
        // =========================================

        theta_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/task2/theta",
                10);


        // =========================================
        // 6. 每 1 ms 更新一次电机状态
        // =========================================

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(
                    &MotorSimulator::update_motor,
                    this));


        RCLCPP_INFO(
            this->get_logger(),
            "Motor simulator started: state update = 1000 Hz");
    }


private:

    // =============================================
    // 接收到新的力矩命令
    // =============================================

    void torque_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        torque_cmd_ = msg->data;
    }


    // =============================================
    // 电机状态更新
    // =============================================

    void update_motor()
    {
        /*
         * 电机机械动力学：
         *
         * J * d(omega)/dt
         * =
         * Te - TL - B * omega
         *
         * 因此：
         *
         * angular_acceleration
         * =
         * (Te - TL - B * omega) / J
         */

        double angular_acceleration =
            (
                torque_cmd_
                - load_torque_
                - B_ * omega_
            ) / J_;


        /*
         * 使用 Euler 数值积分更新角度
         *
         * theta(k+1)
         * =
         * theta(k) + omega(k) * dt
         */

        theta_ =
            theta_ +
            omega_ * dt_;


        /*
         * 更新角速度
         *
         * omega(k+1)
         * =
         * omega(k)
         * +
         * angular_acceleration * dt
         */

        omega_ =
            omega_ +
            angular_acceleration * dt_;


        // =========================================
        // 发布角速度
        // =========================================

        std_msgs::msg::Float64 omega_msg;
        omega_msg.data = omega_;

        omega_publisher_->publish(omega_msg);


        // =========================================
        // 发布当前角度
        // =========================================

        std_msgs::msg::Float64 theta_msg;
        theta_msg.data = theta_;

        theta_publisher_->publish(theta_msg);
    }


    // -------------------------
    // 电机参数
    // -------------------------

    double J_;
    double B_;
    double load_torque_;


    // -------------------------
    // 仿真参数
    // -------------------------

    double simulation_frequency_;
    double dt_;


    // -------------------------
    // 输入
    // -------------------------

    double torque_cmd_ = 0.0;


    // -------------------------
    // 电机状态
    // -------------------------

    double omega_ = 0.0;
    double theta_ = 0.0;


    // -------------------------
    // ROS2 objects
    // -------------------------

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        torque_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        omega_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        theta_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<MotorSimulator>());

    rclcpp::shutdown();

    return 0;
}
