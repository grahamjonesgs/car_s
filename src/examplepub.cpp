    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("my_publisher");
      auto publisher = node->create_publisher<std_msgs::msg::String>("my_topic", 10); // 10 is the QoS profile
      auto message = std_msgs::msg::String();
      message.data = "Hello, ROS 2!";
      while (rclcpp::ok()) {
        publisher->publish(message);
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
      return 0;
    }
