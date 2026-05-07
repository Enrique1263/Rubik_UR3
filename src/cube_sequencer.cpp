#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

enum Face { U = 0, R = 1, F = 2, D = 3, L = 4, B = 5 };

class RubikSequencer : public rclcpp::Node {
public:
    RubikSequencer() : Node("rubik_sequencer") {
        solution_sub = this->create_subscription<std_msgs::msg::String>(
            "/cube_solution", 10, std::bind(&RubikSequencer::process_solution, this, _1));
        
        // Este tópico ahora publica la ráfaga completa de comandos
        sequence_pub = this->create_publisher<std_msgs::msg::String>("/cube_command_sequence", 10);
        
        RCLCPP_INFO(this->get_logger(), "Sequencer de secuencia completa iniciado.");
    }

private:
    struct Orientation { int up, down, front, back, left, right; };
    Orientation current;

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
        // Al rotar en Y (horario), el cubo cae hacia la derecha:
        // El nuevo UP era el de la IZQUIERDA, el nuevo RIGHT era el de ARRIBA...
        Orientation old = current;
        current.up    = old.left;
        current.right = old.up;
        current.down  = old.right;
        current.left  = old.down;
    }

    // Usamos estas para buscar el camino más corto
    void virtual_X_inv() { for(int i=0; i<3; i++) virtual_X(); }
    void virtual_Y_inv() { for(int i=0; i<3; i++) virtual_Y(); }

    void process_solution(const std_msgs::msg::String::SharedPtr msg) {
        std::stringstream ss(msg->data);
        std::string move;
        std::string full_sequence = "";
        
        reset_orientation(); // Siempre empezamos desde la pose actual del simulador

        while (ss >> move) {
            char target_char = move[0];
            int target_face;
            
            // Mapeo de la letra de la solución a la "cara física" original
            if(target_char == 'U') target_face = U;
            else if(target_char == 'R') target_face = R;
            else if(target_char == 'F') target_face = F;
            else if(target_char == 'D') target_face = D;
            else if(target_char == 'L') target_face = L;
            else if(target_char == 'B') target_face = B;
            // RCLCPP_INFO(this->get_logger(), "Procesando movimiento: %s (cara objetivo: %d) current: U=%d, R=%d, F=%d, D=%d, L=%d, B=%d", move.c_str(), target_face, current.up, current.right, current.front, current.down, current.left, current.back);

            // --- ESTRATEGIA DE BÚSQUEDA ---
            // Queremos que target_face termine en current.down
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

            // RCLCPP_INFO(this->get_logger(), "Después de orientar: current: U=%d, R=%d, F=%d, D=%d, L=%d, B=%d", current.up, current.right, current.front, current.down, current.left, current.back);

            // Ahora que la cara está en DOWN, giramos las capas superiores (UM)
            // que equivale a girar la cara DOWN.
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

        // Publicar la secuencia completa
        auto out_msg = std_msgs::msg::String();
        out_msg.data = full_sequence;
        
        // Limpiar espacio final si existe
        if (!out_msg.data.empty() && out_msg.data.back() == ' ') {
            out_msg.data.pop_back();
        }

        RCLCPP_INFO(this->get_logger(), "Secuencia generada: [%s]", out_msg.data.c_str());
        sequence_pub->publish(out_msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solution_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sequence_pub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RubikSequencer>());
    rclcpp::shutdown();
    return 0;
}