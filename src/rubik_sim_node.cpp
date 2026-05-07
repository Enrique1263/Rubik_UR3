#include <memory>
#include <vector>
#include <string>
#include <deque>
#include <random>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

struct Sticker {
    Eigen::Vector3i pos;
    Eigen::Vector3i normal;
    char color;
};

class RubikSimNode : public rclcpp::Node {
public:
    RubikSimNode() : Node("rubik_sim_node") {
        reset_cube();
        marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/cube_markers", 10);
        move_sub = create_subscription<std_msgs::msg::String>(
            "/cube_command", 10, std::bind(&RubikSimNode::handle_move, this, _1));
        state_sub = create_subscription<std_msgs::msg::String>(
            "/cube_state_raw", 10, std::bind(&RubikSimNode::handle_state, this, _1));
        timer = create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&RubikSimNode::update_and_publish, this));
        state_service = create_service<std_srvs::srv::Trigger>(
            "/get_cube_state", std::bind(&RubikSimNode::get_state_callback, this, _1, _2));
        
        RCLCPP_INFO(this->get_logger(), "Simulador Rubik iniciado");
    }

private:
    std::vector<Sticker> stickers;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr move_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_service;

    /* ---------- Animación y Cola ---------- */
    bool animating = false;
    bool scramble_mode = false;
    std::deque<std::string> internal_move_queue;
    
    Eigen::Vector3i anim_axis;
    std::vector<int> anim_layers;
    float anim_angle = 0;
    float anim_speed_normal = 0.15;
    float anim_speed_fast = 0.6;
    bool anim_cw = true;

    void apply_state_string(std::string s) {
        stickers.clear();
        for (int i = 0; i < 54; i++) {
            int face = i / 9;
            int idx = i % 9;
            int row = idx / 3;
            int col = idx % 3;

            Eigen::Vector3i pos, normal;
            int a = col - 1; 
            int b = 1 - row; 

            switch (face) {
                case 0: // U: Arriba: -Y, Derecha: X
                    pos = {a, b, 1};    normal = {0, 0, 1}; break;
                case 1: // R: Arriba: Z, Derecha: -Y
                    pos = {1, a, b};   normal = {1, 0, 0}; break;
                case 2: // F: Arriba: Z, Derecha: X
                    pos = {a, -1, b};   normal = {0, -1, 0}; break;
                case 3: // D: Arriba: Y, Derecha: X
                    pos = {a, -b, -1};  normal = {0, 0, -1}; break;
                case 4: // L: Arriba: Z, Derecha: Y
                    pos = {-1, -a, b};   normal = {-1, 0, 0}; break;
                case 5: // B: Arriba: Z, Derecha: -X
                    pos = {-a, 1, b};   normal = {0, 1, 0}; break;
            }
            stickers.push_back({pos, normal, s[i]});
        }
    }

    void handle_state(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.length() == 54) apply_state_string(msg->data);
    }

    void reset_cube() {
        std::string solved = "";
        for (char c : {'U', 'R', 'F', 'D', 'L', 'B'}) solved += std::string(9, c);
        apply_state_string(solved);
        internal_move_queue.clear();
    }

    void generate_random_moves(int count) {
        // Reiniciamos el generador para asegurar aleatoriedad real
        std::random_device rd;
        std::mt19937 gen(rd());

        // Definimos los movimientos
        std::vector<std::string> moves = {"UM", "UM'", "X", "X'", "Y", "Y'"};
        
        std::vector<double> weights = {3.0, 3.0, 2.0, 2.0, 2.0, 2.0};
        std::discrete_distribution<> dis(weights.begin(), weights.end());

        std::string last_move = "";

        for(int i = 0; i < count; ++i) {
            std::string current_move;
            bool valid = false;

            while (!valid) {
                current_move = moves[dis(gen)];
                if (last_move.empty()) {
                    valid = true;
                } else {
                    bool is_inverse = false;
                    if (current_move[0] == last_move[0]) {
                        if (current_move.size() != last_move.size()) {
                            is_inverse = true;
                        }
                    }
                    if (!is_inverse) valid = true;
                }
            }
            internal_move_queue.push_back(current_move);
            last_move = current_move;
        }
        scramble_mode = true;
        // RCLCPP_INFO(this->get_logger(), "Cola de RANDOM generada con %d movimientos.", count);
    }

    void rotate_layer(Eigen::Vector3i axis, int layer, bool cw) {
        float angle = cw ? -M_PI_2 : M_PI_2;
        Eigen::Matrix3f R = Eigen::AngleAxisf(angle, axis.cast<float>().normalized()).toRotationMatrix();
        for (auto &s : stickers) {
            if (axis.dot(s.pos) == layer) {
                Eigen::Vector3f p = R * s.pos.cast<float>();
                Eigen::Vector3f n = R * s.normal.cast<float>();
                s.pos = p.array().round().cast<int>();
                s.normal = n.array().round().cast<int>();
            }
        }
    }

    void start_animation(std::string cmd) {
        if (animating) return;
        animating = true;
        anim_angle = 0;

        if (cmd == "UM")       { anim_axis = {0,0,1}; anim_layers = {1,0}; anim_cw = true; }
        else if (cmd == "UM'") { anim_axis = {0,0,1}; anim_layers = {1,0}; anim_cw = false; }
        else if (cmd == "X")    { anim_axis = {1,0,0}; anim_layers = {1,0,-1}; anim_cw = true; }
        else if (cmd == "X'")   { anim_axis = {1,0,0}; anim_layers = {1,0,-1}; anim_cw = false; }
        else if (cmd == "Y")    { anim_axis = {0,1,0}; anim_layers = {1,0,-1}; anim_cw = true; }
        else if (cmd == "Y'")   { anim_axis = {0,1,0}; anim_layers = {1,0,-1}; anim_cw = false; }
    }

    void handle_move(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        
        if (cmd == "RESET") { 
            reset_cube(); 
            return; 
        }
        
        if (cmd == "RANDOM") { 
            generate_random_moves(30); 
            // Si no hay nada animándose ahora mismo, forzamos el inicio
            if (!animating && !internal_move_queue.empty()) {
                start_animation(internal_move_queue.front());
                internal_move_queue.pop_front();
            }
            return; 
        }
        
        // Para comandos normales (UM, X, etc.)
        if (!animating && internal_move_queue.empty()) {
            start_animation(cmd);
        } else {
            internal_move_queue.push_back(cmd);
        }
    }

    void update_animation() {
        if (!animating) {
            if (!internal_move_queue.empty()) {
                start_animation(internal_move_queue.front());
                internal_move_queue.pop_front();
            } else {
                scramble_mode = false;
            }
            return;
        }

        float current_speed = scramble_mode ? anim_speed_fast : anim_speed_normal;
        anim_angle += current_speed;

        if (anim_angle >= M_PI_2) {
            for (int l : anim_layers) rotate_layer(anim_axis, l, anim_cw);
            animating = false;
            anim_angle = 0;
        }
    }

    /* ---------- Publicación ---------- */

    std_msgs::msg::ColorRGBA get_color(char c) {
        std_msgs::msg::ColorRGBA col; col.a = 1;
        if (c == 'U') { col.r = 1; col.g = 1; col.b = 1; }      // Blanco
        else if (c == 'D') { col.r = 1; col.g = 1; col.b = 0; } // Amarillo
        else if (c == 'F') { col.r = 1; col.g = 0; col.b = 0; } // ROJO (Front)
        else if (c == 'B') { col.r = 1; col.g = 0.5; col.b = 0; } // NARANJA (Back)
        else if (c == 'L') { col.r = 0; col.g = 1; col.b = 0; } // VERDE (Left)
        else if (c == 'R') { col.r = 0; col.g = 0; col.b = 1; } // AZUL (Right)
        return col;
    }

    void publish_cube() {
        visualization_msgs::msg::MarkerArray ma;
        float cubie = 0.05; float gap = 0.052; float sticker = 0.006;
        int id = 0;

        for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
        for (int z = -1; z <= 1; z++) {
            if (x == 0 && y == 0 && z == 0) continue;
            Eigen::Vector3f pos(x, y, z);
            Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
            if (animating) {
                int layer = anim_axis.dot(Eigen::Vector3i(x, y, z));
                for (int l : anim_layers) {
                    if (layer == l) {
                        float angle = anim_cw ? -anim_angle : anim_angle;
                        R = Eigen::AngleAxisf(angle, anim_axis.cast<float>().normalized()).toRotationMatrix();
                        pos = R * pos;
                        break;
                    }
                }
            }
            pos *= gap;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world"; m.id = id++;
            m.type = m.CUBE; m.pose.position.x = pos.x(); m.pose.position.y = pos.y(); m.pose.position.z = pos.z();
            Eigen::Quaternionf q(R);
            m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
            m.scale.x = cubie; m.scale.y = cubie; m.scale.z = cubie;
            m.color.r = 0.05; m.color.g = 0.05; m.color.b = 0.05; m.color.a = 1;
            ma.markers.push_back(m);
        }

        // --- Dibujar Stickers (Colores) ---
        for (const auto &s : stickers) {
            Eigen::Vector3f pos = s.pos.cast<float>();
            Eigen::Vector3f normal = s.normal.cast<float>();
            Eigen::Vector3f z_local = normal.normalized();
            Eigen::Vector3f x_local;
            if (fabs(z_local.z()) > 0.9) x_local = Eigen::Vector3f(1, 0, 0);
            else x_local = Eigen::Vector3f(0, 0, 1).cross(z_local).normalized();
            Eigen::Vector3f y_local = z_local.cross(x_local).normalized();

            Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
            if (animating) {
                int layer = anim_axis.dot(s.pos);
                for (int l : anim_layers) {
                    if (layer == l) {
                        float angle = anim_cw ? -anim_angle : anim_angle;
                        R = Eigen::AngleAxisf(angle, anim_axis.cast<float>().normalized()).toRotationMatrix();
                        pos = R * pos; z_local = R * z_local; x_local = R * x_local; y_local = R * y_local;
                        break;
                    }
                }
            }
            pos *= gap;
            pos += z_local * (cubie / 2 + sticker / 2);
            Eigen::Matrix3f frame; frame.col(0) = x_local; frame.col(1) = y_local; frame.col(2) = z_local;
            Eigen::Quaternionf q(frame);
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world"; m.id = id++;
            m.type = m.CUBE; m.pose.position.x = pos.x(); m.pose.position.y = pos.y(); m.pose.position.z = pos.z();
            m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
            m.scale.x = cubie * 0.9; m.scale.y = cubie * 0.9; m.scale.z = sticker;
            m.color = get_color(s.color);
            ma.markers.push_back(m);
        }
        marker_pub->publish(ma);
    }

    void get_state_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        if (animating) {
            response->success = false;
            response->message = "Espera a que termine la animación";
            return;
        }

        std::vector<Sticker> stickers_copy = this->stickers;

        // 1. Reorientar la COPIA a posición estándar
        reorient_virtual(stickers_copy);

        // 2. Generar el string de Kociemba usando la copia ya enderezada
        std::string state = "";
        state += scan_face_from_vector({0, 0, 1}, stickers_copy);  // U
        state += scan_face_from_vector({1, 0, 0}, stickers_copy);  // R
        state += scan_face_from_vector({0, -1, 0}, stickers_copy); // F
        state += scan_face_from_vector({0, 0, -1}, stickers_copy); // D
        state += scan_face_from_vector({-1, 0, 0}, stickers_copy); // L
        state += scan_face_from_vector({0, 1, 0}, stickers_copy);  // B

        response->success = true;
        response->message = state;
        RCLCPP_INFO(this->get_logger(), "Estado virtual generado (simulador intacto)");
    }


    void reorient_virtual(std::vector<Sticker> &temp_stickers) {
        // 1. Buscar centro Blanco en la copia
        auto get_center = [&](char color) -> Sticker* {
            for (auto &s : temp_stickers) {
                if (s.color == color && s.pos == s.normal) return &s;
            }
            return nullptr;
        };

        Sticker* cU = get_center('U');
        if (!cU) return;

        // 2. Subir Blanco a Z=1
        while (cU->pos != Eigen::Vector3i(0, 0, 1)) {
            if (cU->pos.x() != 0) rotate_vector(temp_stickers, {0, 1, 0}, true);
            else                  rotate_vector(temp_stickers, {1, 0, 0}, true);
        }

        // 3. Llevar Rojo a Y=-1
        Sticker* cF = get_center('F');
        while (cF->pos != Eigen::Vector3i(0, -1, 0)) {
            rotate_vector(temp_stickers, {0, 0, 1}, true);
        }
    }

    void rotate_vector(std::vector<Sticker> &vec, Eigen::Vector3i axis, bool cw) {
        float angle = cw ? -M_PI_2 : M_PI_2;
        Eigen::Matrix3f R = Eigen::AngleAxisf(angle, axis.cast<float>().normalized()).toRotationMatrix();
        for (auto &s : vec) {
            Eigen::Vector3f p = R * s.pos.cast<float>();
            Eigen::Vector3f n = R * s.normal.cast<float>();
            s.pos = p.array().round().cast<int>();
            s.normal = n.array().round().cast<int>();
        }
    }

    std::string scan_face_from_vector(Eigen::Vector3i normal, const std::vector<Sticker> &vec) {
        std::string face_str = "---------";
        for (const auto &s : vec) {
            if (s.normal.dot(normal) == 1) {
                int u, v;
                if (normal.z() == 1) {        // UP
                    u = s.pos.x() + 1; v = 1 - s.pos.y();
                } else if (normal.z() == -1) { // DOWN
                    u = s.pos.x() + 1; v = s.pos.y() + 1;
                } else if (normal.y() == -1) { // FRONT
                    u = s.pos.x() + 1; v = 1 - s.pos.z();
                } else if (normal.y() == 1) {  // BACK
                    u = 1 - s.pos.x(); v = 1 - s.pos.z();
                } else if (normal.x() == 1) {  // RIGHT
                    u = s.pos.y() + 1; v = 1 - s.pos.z();
                } else if (normal.x() == -1) { // LEFT
                    u = 1 - s.pos.y(); v = 1 - s.pos.z();
                }

                int idx = v * 3 + u;
                if (idx >= 0 && idx < 9) face_str[idx] = s.color;
            }
        }
        return face_str;
    }

    void update_and_publish() {
        update_animation();
        publish_cube();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RubikSimNode>());
    rclcpp::shutdown();
    return 0;
}