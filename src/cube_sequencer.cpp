#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

enum Face { U = 0, R = 1, F = 2, D = 3, L = 4, B = 5 };

class RubikSequencer : public rclcpp::Node {
public:
    RubikSequencer() : Node("rubik_sequencer") {
        solution_sub = this->create_subscription<std_msgs::msg::String>(
            "/cube_solution", 10, std::bind(&RubikSequencer::process_solution, this, _1));
        
        // Publicador que ahora envía los comandos uno a uno al robot
        sequence_pub = this->create_publisher<std_msgs::msg::String>("/cube_command", 10);
        
        // Suscriptor para escuchar el ACK que manda el controlador cuando termina por completo
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_status", 10, std::bind(&RubikSequencer::robot_status_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Sequencer síncrono paso a paso iniciado.");
    }

private:
    struct Orientation { int up, down, front, back, left, right; };
    Orientation current;

    // Variables para la gestión síncrona
    std::queue<std::string> pending_sequence_;
    std::string active_command_;
    bool execution_in_progress_ = false;

    // ROS Comunicaciones adicionales
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solution_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sequence_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

    void reset_orientation() {
        // Orientación inicial estándar (Blanco arriba, Rojo frente)
        current = {U, D, F, B, L, R};
    }

    void virtual_UM(bool clockwise) {
        Orientation old = current;
        if (clockwise) {
            current.front = old.right;
            current.left  = old.front;
            current.back  = old.left;
            current.right = old.back;
        } else {
            current.front = old.left;
            current.left  = old.back;
            current.back  = old.right;
            current.right = old.front;
        }
    }

    void virtual_X() {
        Orientation old = current;
        current.up    = old.back;
        current.front = old.up;
        current.down  = old.front;
        current.back  = old.down;
    }

    void virtual_Y() {
        Orientation old = current;
        current.up    = old.left;
        current.right = old.up;
        current.down  = old.right;
        current.left  = old.down;
    }

    // Funciones inversas virtuales para buscar el camino más corto
    void virtual_X_inv() { for(int i=0; i<3; i++) virtual_X(); }
    void virtual_Y_inv() { for(int i=0; i<3; i++) virtual_Y(); }

    void process_solution(const std_msgs::msg::String::SharedPtr msg) {
        std::stringstream ss(msg->data);
        std::string move;
        std::string full_sequence = "";
        
        reset_orientation(); // Siempre empezamos desde la pose actual

        while (ss >> move) {
            char target_char = move[0];
            int target_face;
            
            // Mapeo de la letra de la solución a la cara física original
            if(target_char == 'U') target_face = U;
            else if(target_char == 'R') target_face = R;
            else if(target_char == 'F') target_face = F;
            else if(target_char == 'D') target_face = D;
            else if(target_char == 'L') target_face = L;
            else if(target_char == 'B') target_face = B;

            // --- ESTRATEGIA DE BÚSQUEDA ---
            // Buscamos que target_face termine en la cara inferior (current.down)
            if (current.down == target_face) {
            } 
            else if (current.front == target_face) {
                full_sequence += "X' "; virtual_X();
            } 
            else if (current.back == target_face) {
                full_sequence += "X "; virtual_X_inv();
            } 
            else if (current.up == target_face) {
                full_sequence += "X X "; virtual_X(); virtual_X();
            } 
            else if (current.left == target_face) {
                full_sequence += "Y "; virtual_Y_inv();
            } 
            else if (current.right == target_face) {
                full_sequence += "Y' "; virtual_Y();
            }

            if (move.find("'") != std::string::npos) {
                full_sequence += "UM' ";
                virtual_UM(false);
            } else if (move.find("2") != std::string::npos) {
                full_sequence += "UM UM ";
                virtual_UM(true); virtual_UM(true);
            } else {
                full_sequence += "UM ";
                virtual_UM(true);
            }
        }

        // --- CARGA SÍNCRONA DE LA SECUENCIA ---
        // Vaciamos cualquier residuo de secuencias anteriores por seguridad
        std::queue<std::string> empty_queue;
        std::swap(pending_sequence_, empty_queue);

        // Troceamos el string resultante y lo añadimos a nuestra cola FIFO
        std::stringstream seq_stream(full_sequence);
        std::string single_cmd;
        while (seq_stream >> single_cmd) {
            pending_sequence_.push(single_cmd);
        }

        RCLCPP_INFO(this->get_logger(), "Secuencia generada con éxito (%zu pasos). Iniciando envío...", pending_sequence_.size());
        
        // Si el robot no está ocupado ejecutando otra cosa, lanzamos el primer paso
        if (!execution_in_progress_ && !pending_sequence_.empty()) {
            execution_in_progress_ = true;
            dispatch_next_command();
        }
    }

    void dispatch_next_command() {
        if (pending_sequence_.empty()) {
            RCLCPP_INFO(this->get_logger(), "¡ENHORABUENA! Toda la secuencia del cubo ha sido ejecutada con éxito.");
            execution_in_progress_ = false;
            active_command_ = "";
            return;
        }

        // Extraemos el primer comando de la cola
        active_command_ = pending_sequence_.front();
        pending_sequence_.pop();

        RCLCPP_INFO(this->get_logger(), "[SEQUENCER] Enviando paso al robot: -> %s (Quedan %zu en cola)", 
                    active_command_.c_str(), pending_sequence_.size());
        
        // Enviamos el comando de forma aislada por el tópico individual
        auto msg = std_msgs::msg::String();
        msg.data = active_command_;
        sequence_pub->publish(msg);
    }

    void robot_status_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!execution_in_progress_) return;

        std::string status = msg->data;
        // El ACK esperado debe coincidir exactamente con lo que el robot acaba de realizar
        std::string expected_ack = "ARRIVED_" + active_command_;

        if (status == expected_ack) {
            RCLCPP_INFO(this->get_logger(), "[ACK COMPLETADO] El robot terminó con éxito el paso: %s", active_command_.c_str());
            // Avanzamos al siguiente comando de la cola de inmediato
            dispatch_next_command();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RubikSequencer>());
    rclcpp::shutdown();
    return 0;
}