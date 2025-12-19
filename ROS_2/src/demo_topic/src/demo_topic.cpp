#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include<chrono>//引入时间相关的头文件
#include<iostream>
using namespace std::chrono_literals;

class Turtle_circle: public rclcpp::Node{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Publisher_;
    public:
        Turtle_circle(const std::string &node_name) : Node(node_name){
            Publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            timer_ = this->create_wall_timer(1000ms, [this]() { this->time_callback(); }); // 为 lambda 表达式
        }
    private:
        void time_callback(){
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 1.0;
            msg.angular.z = 0.5; 
            Publisher_->publish(msg);
            std::cout<<"被调用"<<std::endl;
        }
        
};
int main(int argc,char **argv){
    rclcpp::init(argc,argv); 
    return 0;
}