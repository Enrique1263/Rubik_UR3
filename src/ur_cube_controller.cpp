#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using std::placeholders::_1;

class URCubeController : public rclcpp::Node {
public:
    URCubeController() : Node("ur_cube_controller") {
        
        // Nombres obligatorios en el orden correcto para UR
        joints_ = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        // Publicador directo al controlador del robot
        std::string publish_topic = "/scaled_joint_trajectory_controller/joint_trajectory";
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(publish_topic, 10);

        // Publicador para enviar el ACK al nodo de visión
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

        // Suscriptor para la posición actual
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&URCubeController::joint_state_callback, this, _1));

        // Suscriptor a tus comandos
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cube_command", 10, std::bind(&URCubeController::handle_command, this, _1));

        RCLCPP_INFO(this->get_logger(), "Controlador Global listo. Esperando comandos en /cube_command...");
    }

private:
    void handle_command(const std_msgs::msg::String::SharedPtr msg) {
        if (!joint_state_msg_received_) {
            RCLCPP_WARN(this->get_logger(), "Aún no se ha recibido la posición del robot. Ignorando comando.");
            return;
        }

        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "Ejecutando secuencia GLOBAL para comando: %s", cmd.c_str());

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joints_;
        traj.header.stamp = this->get_clock()->now(); // CRUCIAL para que el driver lo acepte

        // Punto inicial: posición actual
        traj.points.push_back(create_global_point(current_positions_, 0.0));

        double step_time = 2.0; 
        std::vector<double> final_point; // Para guardar el último punto de la secuencia

        if (cmd == "UM") {
            traj.points.push_back(create_global_point({0.0, -1.57, 1.0, -1.0, 1.57, 0.0}, step_time * 1));
            traj.points.push_back(create_global_point({0.0, -1.57, 1.0, -1.0, 1.57, 1.57}, step_time * 2));
            final_point = {0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
            traj.points.push_back(create_global_point(final_point, step_time * 3));
        } 
        else if (cmd == "UM'") {
            traj.points.push_back(create_global_point({0.0, -1.57, 1.0, -1.0, 1.57, 0.0}, step_time * 1));
            traj.points.push_back(create_global_point({0.0, -1.57, 1.0, -1.0, 1.57, -1.57}, step_time * 2));
            final_point = {0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
            traj.points.push_back(create_global_point(final_point, step_time * 3));
        }
        else if (cmd == "X") {
            traj.points.push_back(create_global_point({0.1, -1.4, 0.8, -0.8, 1.5, 0.0}, step_time * 1));
            final_point = {0.1, -1.4, 0.8, -0.8, 1.5, 1.57};
            traj.points.push_back(create_global_point(final_point, step_time * 2));
        }
        else if (cmd == "X'") {
            traj.points.push_back(create_global_point({0.1, -1.4, 0.8, -0.8, 1.5, 0.0}, step_time * 1));
            final_point = {0.1, -1.4, 0.8, -0.8, 1.5, -1.57};
            traj.points.push_back(create_global_point(final_point, step_time * 2));
        }
        else if (cmd == "Y") {
            final_point = {2.199, -1.449, -1.885, -1.588, 2.513, 0.0};
            traj.points.push_back(create_global_point(final_point, step_time * 1));
        }
        else if (cmd == "Y'") {
            final_point = {-0.1, -1.6, 1.2, -1.2, 1.5, -1.57};
            traj.points.push_back(create_global_point(final_point, step_time * 1));
        }
        else if (cmd == "RESET") {
            final_point = {0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
            traj.points.push_back(create_global_point(final_point, step_time * 2));
        }
        else if (cmd == "PICTURE") {
            final_point = {-4.03, -1.83, -1.64, -1.23, 2.41, 4.72};
            traj.points.push_back(create_global_point(final_point, step_time * 1));
        }
        else if (cmd == "SAFE") {
            final_point = {0.0, -1.90, 0.0, -1.57, -1.57, 0.0};
            traj.points.push_back(create_global_point(final_point, step_time * 1));
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Comando desconocido: %s", cmd.c_str());
            return;
        }

        if (traj.points.size() > 1) {
            target_positions_ = final_point;
            last_command_ = cmd;
            is_moving_ = true;
            publisher_->publish(traj);
        } else {
            RCLCPP_WARN(this->get_logger(), "Comando desconocido o sin secuencia definida: %s", cmd.c_str());
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<double> ordered_pos(6, 0.0);
        bool found_all = true;

        for (size_t i = 0; i < joints_.size(); ++i) {
            bool found = false;
            for (size_t j = 0; j < msg->name.size(); ++j) {
                if (joints_[i] == msg->name[j]) {
                    ordered_pos[i] = msg->position[j];
                    found = true;
                    break;
                }
            }
            if (!found) found_all = false;
        }

        if (found_all) {
            current_positions_ = ordered_pos;
            joint_state_msg_received_ = true;

            // Lógica de ACK: Comprobar si hemos llegado al objetivo
            if (is_moving_ && !target_positions_.empty()) {
                double total_error = 0.0;
                for (size_t i = 0; i < 6; ++i) {
                    total_error += std::pow(current_positions_[i] - target_positions_[i], 2);
                }
                
                if (std::sqrt(total_error) < 0.005) { // Tolerancia de llegada
                    is_moving_ = false;
                    auto ack_msg = std_msgs::msg::String();
                    ack_msg.data = "ARRIVED_" + last_command_;
                    status_pub_->publish(ack_msg);
                    RCLCPP_INFO(this->get_logger(), "[ACK] Robot en posición para comando: %s", last_command_.c_str());
                }
            }
        }
    }

    trajectory_msgs::msg::JointTrajectoryPoint create_global_point(const std::vector<double>& pos, double time_sec) {
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = pos;
        p.time_from_start = rclcpp::Duration::from_seconds(time_sec);
        return p;
    }

    // Variables
    std::vector<std::string> joints_;
    std::vector<double> current_positions_;
    std::vector<double> target_positions_;
    std::string last_command_;
    bool joint_state_msg_received_ = false;
    bool is_moving_ = false;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<URCubeController>());
    rclcpp::shutdown();
    return 0;
}