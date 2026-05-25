#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "ur_msgs/srv/set_io.hpp"

using std::placeholders::_1;

// Tipos de acciones internas para la mini máquina de estados
enum class ActionType { MOVE, GRIPPER };

struct SubTask {
    ActionType type;
    std::vector<double> target_pose; // Solo para MOVE
    bool gripper_open;               // Solo para GRIPPER
    double duration;
};

class URCubeController : public rclcpp::Node {
public:
    URCubeController() : Node("ur_cube_controller") {
        
        joints_ = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        status_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&URCubeController::joint_state_callback, this, _1));

        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cube_command", 10, std::bind(&URCubeController::handle_command, this, _1));

        io_client_ = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

        RCLCPP_INFO(this->get_logger(), "Controlador Global con Máquina de Estados listo.");
    }

private:
    void handle_command(const std_msgs::msg::String::SharedPtr msg) {
        if (!joint_state_msg_received_) {
            RCLCPP_WARN(this->get_logger(), "Aún no se ha recibido la posición del robot. Ignorando comando.");
            return;
        }
        if (is_busy_) {
            RCLCPP_WARN(this->get_logger(), "El robot está ejecutando otra secuencia. Comando '%s' ignorado.", msg->data.c_str());
            return;
        }

        std::string cmd = msg->data;
        last_command_ = cmd;
        RCLCPP_INFO(this->get_logger(), "Ejecutando secuencia GLOBAL para comando: %s", cmd.c_str());

        // Limpiamos la cola de tareas anteriores
        std::queue<SubTask> empty_queue;
        std::swap(task_queue_, empty_queue);

        if (cmd == "UM") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.1421162523654, -1.8610445814016, -1.6013395887048, -1.2503538761287, 1.5706217938697, 0.034732052114687}, false, 1.0});
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.1421162523654, -1.8610445814016, -1.6013395887048, -1.2503538761287, 1.5706217938697, 1.6057029118348}, false, 1.0});
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 1.6057029118348}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 1.0});
        } 
        else if (cmd == "UM'") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 1.6057029118348}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.1421162523654, -1.8610445814016, -1.6013395887048, -1.2503538761287, 1.5706217938697, 1.6057029118348}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.1421162523654, -1.8610445814016, -1.6013395887048, -1.2503538761287, 1.5706217938697, 0.01490658504}, false, 1.0});
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.01490658504}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 1.0});
        }
        else if (cmd == "X'") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.1421162523654, -1.8610445814016, -1.6013395887048, -1.2503538761287, 1.5706217938697, 0.034732052114687}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.139149192637, -1.6179202165987, -2.6221826681963, -2.0561723917745, -1.5709708597201, 3.1389746597118}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.1489230364, -1.8788469397719, -2.6675612287, -1.7498671080, -1.5812683023, 3.1370547975}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.1496211681, -2.0298179201, -2.6768114737, -1.5896458827, -1.5826645657, 3.1361821329}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.1489230364, -1.8788469397719, -2.6675612287, -1.7498671080, -1.5812683023, 3.1370547975}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.139149192637, -1.6179202165987, -2.6221826681963, -2.0561723917745, -1.5709708597201, 3.1389746597118}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.1412435877, -1.6765632794, -1.4767230801, -1.8137461586, 0.9944886077, 0.756425697814}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 0.4});

        }
        else if (cmd == "X") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.1412435877, -1.6765632794, -1.4767230801, -1.8137461586, 0.9944886077, 0.756425697814}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.139149192637, -1.6179202165987, -2.6221826681963, -2.0561723917745, -1.5709708597201, 3.1389746597118}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.1489230364, -1.8788469397719, -2.6675612287, -1.7498671080, -1.5812683023, 3.1370547975}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.1496211681, -2.0298179201, -2.6768114737, -1.5896458827, -1.5826645657, 3.1361821329}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.1489230364, -1.8788469397719, -2.6675612287, -1.7498671080, -1.5812683023, 3.1370547975}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.139149192637, -1.6179202165987, -2.6221826681963, -2.0561723917745, -1.5709708597201, 3.1389746597118}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.1438615816174, -1.8528415339172, -1.6140804922444, -1.2459905529988, 1.5706217938697, 0.033161255787892}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 0.4});
        }
        else if (cmd == "Y'") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-2.739468794, -1.799958058, -1.285784060, -2.385166956, 0.9150761268, 2.026152729}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-2.4743532805, -2.4048891763, -1.2592550553, -2.6214845364, 0.669159235214, 3.1461305096}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-2.4745278134, -2.5445155164825, -1.291718179401, -2.4513149344, 0.668635636439, 3.146654108420}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
            task_queue_.push({ActionType::MOVE, {-2.4783675378319, -2.4239132651697, -1.2700760966763, -2.5916394062864, 0.66514497793504, 3.1463050425702}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-2.8036969104037, -2.0755455464717, -1.1328932174695, -2.3038346126325, 1.1107275359692, 1.6166984861223}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.1333896061054, -1.8114772306449, -1.4357078426905, -1.4653784399744, 1.5707963267949, 1.5952309363228}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.133913204881, -1.8528415339172, -1.6104153008152, -1.2491321456523, 1.5706217938697, 1.5950564033976}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 1.6057029118348}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 1.0});
        }
        else if (cmd == "Y") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 2.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 1.6057029118348}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.1421162523654, -1.8610445814016, -1.6013395887048, -1.2503538761287, 1.5706217938697, 1.6057029118348}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 1.6057029118348}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-2.8036969104037, -2.0755455464717, -1.1328932174695, -2.3038346126325, 1.1107275359692, 1.6166984861223}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-2.4783675378319, -2.4239132651697, -1.2700760966763, -2.5916394062864, 0.66514497793504, 3.1463050425702}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-2.4783675378319, -2.5377087323998, -1.2932889757278, -2.45480559293, 0.66462137915944, 3.1466541084206}, false, 0.4});
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
            task_queue_.push({ActionType::MOVE, {-2.4783675378319, -2.4239132651697, -1.2700760966763, -2.5916394062864, 0.66514497793504, 3.1463050425702}, false, 0.4});
            task_queue_.push({ActionType::MOVE, {-2.739468794, -1.799958058, -1.285784060, -2.385166956, 0.9150761268, 2.026152729}, false, 1.0});
            task_queue_.push({ActionType::MOVE, {-3.141767186515, -1.8181094818025, -1.4114477660878, -1.4830062654196, 1.5707963267949, 0.034906585039887}, false, 1.0});
        }
        else if (cmd == "RESET") {
            task_queue_.push({ActionType::MOVE, {0.0, -1.57, 0.0, 0.0, 0.0, 0.0}, false, 2.0});
        }
        else if (cmd == "PICTURE") {
            task_queue_.push({ActionType::MOVE, {-3.5657076618, -1.75405589825, -1.4498450096, -1.4498450096, 1.8760544129, 0.601689526}, false, 3.0});
            task_queue_.push({ActionType::MOVE, {-4.01530448, -1.86139365, -1.83050132, -1.123643, 2.50437294, -1.60343398718}, false, 2.0});
        }
        else if (cmd == "SAFE") {
            task_queue_.push({ActionType::MOVE, {-3.14, -1.57, 0.0, -1.57, 3.14, 0.0}, false, 5.0});
        }
        else if (cmd == "OPEN") {
            task_queue_.push({ActionType::GRIPPER, {}, true, 0.0});
        }
        else if (cmd == "CLOSE") {
            task_queue_.push({ActionType::GRIPPER, {}, false, 0.0});
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Comando desconocido: %s", cmd.c_str());
            return;
        }

        // Arrancamos la cola
        if (!task_queue_.empty()) {
            is_busy_ = true;
            execute_next_sub_task();
        }
    }

    void execute_next_sub_task() {
        if (task_queue_.empty()) {
            is_busy_ = false;
            auto ack_msg = std_msgs::msg::String();
            ack_msg.data = "ARRIVED_" + last_command_;
            status_pub_->publish(ack_msg);
            RCLCPP_INFO(this->get_logger(), "[ACK FINAL] Secuencia completa terminada para: %s", last_command_.c_str());
            return;
        }

        current_task_ = task_queue_.front();
        task_queue_.pop();

        if (current_task_.type == ActionType::MOVE) {
            is_waiting_for_joints_ = true;
            target_positions_ = current_task_.target_pose;

            trajectory_msgs::msg::JointTrajectory traj;
            traj.joint_names = joints_;
            traj.points.push_back(create_global_point(current_positions_, 0.0));
            
            traj.points.push_back(create_global_point(target_positions_, current_task_.duration));
            
            publisher_->publish(traj);
        }
        else if (current_task_.type == ActionType::GRIPPER) {
            is_waiting_for_gripper_ = true;
            actuate_gripper(current_task_.gripper_open);

            // Mantenemos la pausa de cortesía para que actúe la pinza física
            gripper_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(600),
                std::bind(&URCubeController::gripper_timer_callback, this));
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

            // Monitorización del movimiento del sub-paso actual
            if (is_busy_ && is_waiting_for_joints_ && !target_positions_.empty()) {
                double total_error = 0.0;
                for (size_t i = 0; i < 6; ++i) {
                    total_error += std::pow(current_positions_[i] - target_positions_[i], 2);
                }
                
                if (std::sqrt(total_error) < 0.005) { 
                    is_waiting_for_joints_ = false;
                    RCLCPP_INFO(this->get_logger(), "Sub-punto alcanzado. Avanzando secuencia...");
                    execute_next_sub_task(); // Ejecuta el siguiente elemento en la cola
                }
            }
        }
    }

    void gripper_timer_callback() {
        gripper_timer_->cancel(); // Detener el timer para que no vuelva a saltar
        if (is_busy_ && is_waiting_for_gripper_) {
            is_waiting_for_gripper_ = false;
            RCLCPP_INFO(this->get_logger(), "Pausa de pinza completada. Avanzando secuencia...");
            execute_next_sub_task(); // Continuamos con el siguiente paso de la cola
        }
    }

    void actuate_gripper(bool open) {
        auto req0 = std::make_shared<ur_msgs::srv::SetIO::Request>();
        req0->fun = 1;    
        req0->pin = 16;    
        req0->state = open ? 0.0 : 1.0;

        auto req1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
        req1->fun = 1;
        req1->pin = 17;    
        req1->state = open ? 1.0 : 0.0;

        io_client_->async_send_request(req0);
        io_client_->async_send_request(req1);
    }

    trajectory_msgs::msg::JointTrajectoryPoint create_global_point(const std::vector<double>& pos, double time_sec) {
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = pos;
        p.velocities = std::vector<double>(6, 0.0); 
        p.accelerations = std::vector<double>(6, 0.0);
        p.time_from_start = rclcpp::Duration::from_seconds(time_sec);
        return p;
    }

    // Variables de control del robot y flujos
    std::vector<std::string> joints_;
    std::vector<double> current_positions_;
    std::vector<double> target_positions_;
    std::string last_command_;
    bool joint_state_msg_received_ = false;
    
    // Variables de control de la máquina de estados
    bool is_busy_ = false;
    bool is_waiting_for_joints_ = false;
    bool is_waiting_for_gripper_ = false;
    
    std::queue<SubTask> task_queue_;
    SubTask current_task_;
    rclcpp::TimerBase::SharedPtr gripper_timer_;

    // ROS Comunicaciones
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<URCubeController>());
    rclcpp::shutdown();
    return 0;
}