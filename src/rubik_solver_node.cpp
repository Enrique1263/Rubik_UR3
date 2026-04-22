#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// La librería está escrita en C, por lo que usamos extern "C"
extern "C" {
    #include "search.h"
}

using std::placeholders::_1;

class RubikSolverNode : public rclcpp::Node {
public:
    RubikSolverNode() : Node("rubik_solver_node") {
        // Suscribirse al estado raw del cubo (los 54 caracteres)
        state_sub = this->create_subscription<std_msgs::msg::String>(
            "/cube_state_raw", 10, std::bind(&RubikSolverNode::solve_callback, this, _1));

        // Publicar la solución teórica (Movimientos WCA)
        solution_pub = this->create_publisher<std_msgs::msg::String>("/cube_solution", 10);

        RCLCPP_INFO(this->get_logger(), "Solver Kociemba iniciado y listo.");
    }

private:
    void solve_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string raw_state = msg->data;

        if (raw_state.length() != 54) {
            RCLCPP_ERROR(this->get_logger(), "Estado inválido: se recibieron %zu caracteres, se esperan 54.", raw_state.length());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Calculando solución para: %s", raw_state.c_str());

        /* Función solution de ckociemba:
           1. facelets: string de 54 caracteres (U..R..F..D..L..B)
           2. maxDepth: 20-24 es lo ideal.
           3. timeout: tiempo máximo de búsqueda en ms.
           4. useSeparator: 0 para devolver "U2 R", 1 para "U2 R "
           5. cacheDir: Directorio donde buscar/generar tablas de poda.
        */
        char *result = solution(
            const_cast<char*>(raw_state.c_str()),
            24,
            1000,
            0,
            "./cache" 
        );

        if (result == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Kociemba no pudo hallar solución. Revisa el orden de las pegatinas.");
            return;
        }

        auto sol_msg = std_msgs::msg::String();
        sol_msg.data = std::string(result);
        
        RCLCPP_INFO(this->get_logger(), "Solución: %s", sol_msg.data.c_str());
        solution_pub->publish(sol_msg);

        // MUY IMPORTANTE: La librería ckociemba reserva memoria con malloc
        free(result);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr solution_pub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RubikSolverNode>());
    rclcpp::shutdown();
    return 0;
}