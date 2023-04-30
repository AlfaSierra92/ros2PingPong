// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;

#include "plotter_time/msg/plottime.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node{
    rclcpp::TimerBase::SharedPtr timer_; //timer per il create_wall_timer
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; //publisher per Ping-pong
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_time; //publihser per rqt_plot
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dim;
    rclcpp::Publisher<plotter_time::msg::Plottime>::SharedPtr publisher_time;
    size_t count_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    rcl_time_point_value_t t_pong;

public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0){

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic1", 10);
    //publisher_time = this->create_publisher<std_msgs::msg::Float32>("time", 10);
    publisher_time = this->create_publisher<plotter_time::msg::Plottime>("time", 10);
    //publisher_dim = this->create_publisher<std_msgs::msg::Float32>("dim", 10);
    timer_ = this->create_wall_timer(
      4000ms, std::bind(&MinimalPublisher::timer_callback, this));

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic2", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
      
  }

private:
  void timer_callback(){ //funzione richiamata da publisher

    auto message = std_msgs::msg::String();
    message.data = /*"Ping " + */std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    for(size_t i = 0;i<count_;i++){
      message.data = message.data + "0000000000000000000000000000000000000";
    }
    publisher_->publish(message);

    //timer stuff
    rclcpp::Clock *clk = new rclcpp::Clock();
    rclcpp::Time time = clk->now();
    rcl_time_point_value_t ns = time.nanoseconds(); // Uint64_t in ref implementation
    t_pong = ns;
    RCLCPP_INFO(this->get_logger(), "Time is: %lld", ns);
  }

  //funzione richiamata da subscriber
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    rclcpp::Clock *clk = new rclcpp::Clock();
    rclcpp::Time time = clk->now();
    rcl_time_point_value_t ns = time.nanoseconds(); // Uint64_t in ref implementation
    rcl_time_point_value_t t2 = ns - t_pong;
    //float tmp = (float) t2/1000000;

    RCLCPP_INFO(this->get_logger(), "TIME PASSED: %f", (float) t2/1000000);
    //auto message_time = std_msgs::msg::Float32();
    //message_time.data = (float) t2/1000000;
    //publisher_time->publish(message_time);
    auto message_time = plotter_time::msg::Plottime();
    message_time.time = (float) t2/1000000;
    message_time.dim = 37*count_;
    publisher_time->publish(message_time);
  }

};

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
