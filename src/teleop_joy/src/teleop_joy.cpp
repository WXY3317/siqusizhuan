/**
 ******************************************************************************
 * File Name          : teleop_joy.cpp
 * Description       : teleop_joy program body
 ******************************************************************************
 *
 * Copyright (c) 2019 HopeMotion Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of HopeMotion nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for HopeMotion.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY HOPEMOTION AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL HOPEMOTION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <teleop_joy/teleop_joy.h>

class Teleop : public rclcpp::Node
{
public:
    Teleop();

private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void on_timer();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_steer_z_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_steer_y_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lifter_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_;
    std_msgs::msg::UInt16 steer_z_;
    std_msgs::msg::UInt16 steer_y_;
    std_msgs::msg::Float64 lifter_;

    double v_x_{0.7}, v_y_{0.3}, v_angular_{0.3};
    int active_{0};
    int old_active_{0};
    int task_flags_[2] = {0, 0};
    int tmp_{0};
    int task_A_{1};
    int task_B_{2};
    int steer_gain_{50};
};

Teleop::Teleop() : rclcpp::Node("teleop_joy")
{
    // 声明并获取参数（ROS2）
    this->declare_parameter<double>("x_speed_scale", 0.7);
    this->declare_parameter<double>("y_speed_scale", 0.3);
    this->declare_parameter<double>("w_speed_scale", 0.3);
    this->declare_parameter<int>("task_a", 1);
    this->declare_parameter<int>("task_b", 2);
    this->declare_parameter<int>("steer_gain", 50);
    this->declare_parameter<int>("usr_task", 0);

    (void)this->get_parameter("x_speed_scale", v_x_);
    (void)this->get_parameter("y_speed_scale", v_y_);
    (void)this->get_parameter("w_speed_scale", v_angular_);
    (void)this->get_parameter("task_a", task_A_);
    (void)this->get_parameter("task_b", task_B_);
    (void)this->get_parameter("steer_gain", steer_gain_);

    steer_z_.data = 500;
    steer_y_.data = 500;
    lifter_.data = 0.0;

    // 发布者（ROS2）
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));
    pub_steer_z_ = this->create_publisher<std_msgs::msg::UInt16>("steer_z", rclcpp::QoS(10));
    pub_steer_y_ = this->create_publisher<std_msgs::msg::UInt16>("steer_y", rclcpp::QoS(10));
    pub_lifter_ = this->create_publisher<std_msgs::msg::Float64>("lifter", rclcpp::QoS(10));

    // 订阅者（ROS2）
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        rclcpp::QoS(10),
        std::bind(&Teleop::callback, this, std::placeholders::_1));

    // 定时器替代 ros::Rate 循环
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Teleop::on_timer, this));
}

void Teleop::on_timer()
{
    if (active_)
    {  
        pub_cmd_vel_->publish(twist_);
        old_active_ = 1;
    }
    else
    {
        if (old_active_)
        {  
            twist_.linear.x = 0.0;
            twist_.linear.y = 0.0;
            twist_.angular.z = 0.0;
            pub_cmd_vel_->publish(twist_);
        }
        old_active_ = active_;
    }
}

void Teleop::callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    twist_.linear.x = (joy->axes[4]) * v_x_;
    twist_.linear.y = (joy->axes[3]) * v_y_;
    twist_.angular.z = (joy->axes[0]) * v_angular_;

    active_ = (joy->buttons[4] == 1) ? 1 : 0;

    if (joy->buttons[0] == 1)
    {
        task_flags_[0] = 1;
        RCLCPP_INFO(this->get_logger(), "[TELEOP]: 设置任务A[%d]", task_A_);
    }

    if (joy->buttons[1] == 1)
    {
        task_flags_[1] = 1;
        RCLCPP_INFO(this->get_logger(), "[TELEOP]: 设置任务B[%d]", task_B_);
    }

    if (joy->buttons[1] == 1)
    {
        if ((task_flags_[1] == 0) && (task_flags_[0] == 0))
        {
            RCLCPP_INFO(this->get_logger(), "[TELEOP]: 请按[A B]键设置任务");
        }
        else
        {
            // 读取并更新本节点参数 usr_task（ROS2）
            (void)this->get_parameter("usr_task", tmp_);

            if (task_flags_[0])
            {
                task_flags_[0] = 0;
                if (task_A_ == tmp_)
                {
                    RCLCPP_INFO(this->get_logger(), "[TELEOP]: 任务A已经存在[%d]", task_A_);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "[TELEOP]: 执行任务A[%d]", task_A_);
                    this->set_parameter(rclcpp::Parameter("usr_task", task_A_));
                }
            }

            if (task_flags_[1])
            {
                task_flags_[1] = 0;
                (void)this->get_parameter("usr_task", tmp_);
                if (task_B_ == tmp_)
                {
                    RCLCPP_INFO(this->get_logger(), "[TELEOP]: 任务B已经存在[%d]", task_B_);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "[TELEOP]: 执行任务B[%d]", task_B_);
                    this->set_parameter(rclcpp::Parameter("usr_task", task_B_));
                }
            }
        }
    }

    if (joy->buttons[3] == 1)
    {
        RCLCPP_INFO(this->get_logger(), "[TELEOP]: 取消任务");
        tmp_ = 0;
        this->set_parameter(rclcpp::Parameter("usr_task", tmp_));
    }

    if (joy->buttons[6] == 1)
    {
        // left,right
        if (joy->axes[0] == 1)
        {
            if (steer_z_.data >= 1000)
            {
                steer_z_.data = 1000;
            }
            else
            {
                steer_z_.data += steer_gain_;
            }
            pub_steer_z_->publish(steer_z_);
        }
        else if (joy->axes[0] == -1)
        {
            if (steer_z_.data <= 0)
            {
                steer_z_.data = 0;
            }
            else
            {
                steer_z_.data -= steer_gain_;
            }
            pub_steer_z_->publish(steer_z_);
        }

        // up,down
        if (joy->axes[3] == -1)
        {
            if (steer_y_.data >= 1000)
            {
                steer_y_.data = 1000;
            }
            else
            {
                steer_y_.data += steer_gain_;
            }
            pub_steer_y_->publish(steer_y_);
        }
        else if (joy->axes[3] == 1)
        {
            if (steer_y_.data <= 0)
            {
                steer_y_.data = 0;
            }
            else
            {
                steer_y_.data -= steer_gain_;
            }
            pub_steer_y_->publish(steer_y_);
        }
    }
    else
    {
        if (joy->axes[7] == 1)
        {
            lifter_.data  = 1;
            pub_lifter_->publish(lifter_);
        }
        else if (joy->axes[7] == -1)
        {
            lifter_.data  = -1;
            pub_lifter_->publish(lifter_);
        }else
        {
            lifter_.data  = 0;
            pub_lifter_->publish(lifter_);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    return 0;
}
