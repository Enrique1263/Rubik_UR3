#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RubikSimController : public rclcpp::Node {
public:
    RubikSimController() : Node("rubik_sim_controller") {
        command_pub = this->create_publisher<std_msgs::msg::String>("/cube_command", 10);
        sequence_sub = this->create_subscription<std_msgs::msg::String>(
            "/cube_command_sequence", 10, std::bind(&RubikSimController::handle_sequence, this, std::placeholders::_1));

        timer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RubikSimController::process_queue, this));

        RCLCPP_INFO(this->get_logger(), "Controlador de simulación iniciado.");
    }

private:
    void handle_sequence(const std_msgs::msg::String::SharedPtr msg) {
        std::stringstream ss(msg->data);
        std::string cmd;
        rclcpp::sleep_for(std::chrono::seconds(2));
        while (ss >> cmd) {
            move_queue.push_back(cmd);
        }
    }

    void process_queue() {
        if (!move_queue.empty()) {
            auto msg = std_msgs::msg::String();
            msg.data = move_queue.front();
            move_queue.pop_front();
            command_pub->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sequence_sub;
    rclcpp::TimerBase::SharedPtr timer;
    std::deque<std::string> move_queue;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RubikSimController>());
    rclcpp::shutdown();
    return 0;
}