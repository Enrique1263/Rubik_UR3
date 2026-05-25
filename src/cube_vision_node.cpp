#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

#include "cube_vision_core_img.hpp"

#define CUBE_VISION_RUN_ROBOT_AND_CAMERA 1

class CubeVisionNode : public rclcpp::Node {
public:
    CubeVisionNode() : Node("cube_vision_node") {
        std::string pkg_share_dir;
        try {
            pkg_share_dir = ament_index_cpp::get_package_share_directory("rubik_viz");
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "No se pudo encontrar el directorio share del paquete: %s", e.what());
            return;
        }

        declare_parameter<std::string>("image_1_path", pkg_share_dir + "/imagenes/imagen1.jpeg");
        declare_parameter<std::string>("image_2_path", pkg_share_dir + "/imagenes/imagen2.jpeg");
        declare_parameter<std::string>("circles_config_1_path", pkg_share_dir + "/imagenes/circleBottom.json");
        declare_parameter<std::string>("circles_config_2_path", pkg_share_dir + "/imagenes/circleTop.json");

        declare_parameter<std::string>("view_command", "PICTURE");
        declare_parameter<std::string>("image_topic", "/image_raw");
        declare_parameter<std::string>("cube_state_topic", "/cube_state_raw");
        declare_parameter<std::string>("robot_command_topic", "/cube_command");
        declare_parameter<std::string>("robot_status_topic", "/robot_status");
        declare_parameter<int>("settle_ms_after_arrival", 2000);
        declare_parameter<bool>("write_debug", true);

        image_1_path_ = get_parameter("image_1_path").as_string();
        image_2_path_ = get_parameter("image_2_path").as_string();
        circles_config_1_path_ = get_parameter("circles_config_1_path").as_string();
        circles_config_2_path_ = get_parameter("circles_config_2_path").as_string();
        view_command_ = get_parameter("view_command").as_string();
        settle_ms_after_arrival_ = get_parameter("settle_ms_after_arrival").as_int();
        write_debug_ = get_parameter("write_debug").as_bool();

        state_pub_ = create_publisher<std_msgs::msg::String>(
            get_parameter("cube_state_topic").as_string(), 10);
        debug_pub_ = create_publisher<std_msgs::msg::String>("/cube_vision_debug", 10);

#if CUBE_VISION_RUN_ROBOT_AND_CAMERA
        command_pub_ = create_publisher<std_msgs::msg::String>(
            get_parameter("robot_command_topic").as_string(), 10);

        status_sub_ = create_subscription<std_msgs::msg::String>(
            get_parameter("robot_status_topic").as_string(), 10,
            std::bind(&CubeVisionNode::status_callback, this, std::placeholders::_1));

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            get_parameter("image_topic").as_string(), 10,
            std::bind(&CubeVisionNode::image_callback, this, std::placeholders::_1));

        try {
            circles_config_1_ = cube_vision::load_circles_config(circles_config_1_path_);
            circles_config_2_ = cube_vision::load_circles_config(circles_config_2_path_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error cargando circles json: %s", e.what());
            return;
        }

        start_live_sequence();
#else
        offline_timer_ = create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&CubeVisionNode::run_offline_once, this));
#endif

        RCLCPP_INFO(get_logger(), "cube_vision_node listo. Modo live=%d", CUBE_VISION_RUN_ROBOT_AND_CAMERA);
    }

private:
    // Máquina de estados expandida para soportar la secuencia intermedia X -> Y -> X -> PICTURE
    enum class Stage {
        Idle,
        MovingToView1,
        WaitingImage1,
        MovingToX1,
        MovingToY,
        MovingToX2,
        MovingToView2,
        WaitingImage2,
        Done
    };

    Stage stage_ = Stage::Idle;
    bool offline_done_ = false;

    std::string image_1_path_;
    std::string image_2_path_;
    std::string circles_config_1_path_;
    std::string circles_config_2_path_;
    std::string view_command_;
    int settle_ms_after_arrival_ = 2000;
    bool write_debug_ = true;

    std::vector<cube_vision::Circle> circles_config_1_;
    std::vector<cube_vision::Circle> circles_config_2_;
    std::vector<std::string> colors_view_1_;
    std::vector<std::string> colors_view_2_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr offline_timer_;

    std::vector<std::string> extract_27_colors_from_mat(
        const cv::Mat &image_bgr,
        const std::vector<cube_vision::Circle> &circles,
        const std::string &debug_output_path = "") const
    {
        if (circles.size() != 27) {
            throw std::runtime_error("El circles_config debe tener 27 puntos. Tiene: " +
                                     std::to_string(circles.size()));
        }

        std::vector<std::string> colors;
        colors.reserve(27);

        cv::Mat debug;
        if (!debug_output_path.empty()) {
            image_bgr.copyTo(debug);
        }

        for (const auto &circle : circles) {
            const cube_vision::StickerSample sample = cube_vision::sample_circle(image_bgr, circle);
            colors.push_back(sample.color);

            if (!debug_output_path.empty()) {
                cv::Scalar draw_color(180, 180, 180);
                if (sample.color == "white") draw_color = cv::Scalar(255, 255, 255);
                else if (sample.color == "yellow") draw_color = cv::Scalar(0, 255, 255);
                else if (sample.color == "red") draw_color = cv::Scalar(0, 0, 255);
                else if (sample.color == "orange") draw_color = cv::Scalar(0, 165, 255);
                else if (sample.color == "green") draw_color = cv::Scalar(0, 255, 0);
                else if (sample.color == "blue") draw_color = cv::Scalar(255, 0, 0);

                cv::circle(debug,
                           cv::Point(static_cast<int>(circle.cx), static_cast<int>(circle.cy)),
                           static_cast<int>(circle.r),
                           draw_color,
                           2);

                cv::putText(debug,
                            std::to_string(circle.id) + ":" + sample.color.substr(0, 1),
                            cv::Point(static_cast<int>(circle.cx - circle.r),
                                      static_cast<int>(circle.cy - circle.r)),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.45,
                            draw_color,
                            1);
            }
        }

        if (!debug_output_path.empty()) {
            cv::imwrite(debug_output_path, debug);
        }

        return colors;
    }

    std::string build_solver_state_from_current_colors() const {
        const auto cube = cube_vision::build_cube_from_two_views(colors_view_1_, colors_view_2_);
        const std::string state = cube_vision::cube_to_solver_string(cube);

        std::string error;
        if (!cube_vision::validate_solver_string(state, &error)) {
            throw std::runtime_error(error);
        }

        return state;
    }

    void publish_state(const std::string &state) {
        auto msg = std_msgs::msg::String();
        msg.data = state;
        state_pub_->publish(msg);

        std::ostringstream oss;
        oss << "SOLVER_STRING=" << state << "\n";
        oss << "LEN=" << state.size() << "\n";
        cube_vision::print_faces(state, oss);

        auto dbg = std_msgs::msg::String();
        dbg.data = oss.str();
        debug_pub_->publish(dbg);

        RCLCPP_INFO(get_logger(), "Estado publicado en /cube_state_raw: %s", state.c_str());
    }

    void run_offline_once() {
        if (offline_done_) return;
        offline_done_ = true;
        offline_timer_->cancel();

        try {
            const std::string state = cube_vision::state_from_two_images(
                image_1_path_, image_2_path_,
                circles_config_1_path_, circles_config_2_path_,
                write_debug_
            );
            publish_state(state);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error en vision offline: %s", e.what());
        }
    }

#if CUBE_VISION_RUN_ROBOT_AND_CAMERA
    void start_live_sequence() {
        stage_ = Stage::MovingToView1;
        rclcpp::sleep_for(std::chrono::seconds(5));
        send_robot_command(view_command_); // Manda "PICTURE"
    }

    void send_robot_command(const std::string &command) {
        auto msg = std_msgs::msg::String();
        msg.data = command;
        command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Comando robot enviado: %s", command.c_str());
    }

    void status_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Robot status: %s", msg->data.c_str());

        // Si el robot no ha terminado su trayectoria actual, no hacemos nada.
        if (msg->data.find("ARRIVED") == std::string::npos) {
            return;
        }

        // Transiciones basadas en la llegada del robot a los puntos objetivos
        switch (stage_) {
            case Stage::MovingToView1:
                rclcpp::sleep_for(std::chrono::milliseconds(settle_ms_after_arrival_));
                stage_ = Stage::WaitingImage1;
                RCLCPP_INFO(get_logger(), "Robot en pose 1. Capturando imagen 1...");
                break;

            case Stage::MovingToX1:
                stage_ = Stage::MovingToY;
                send_robot_command("Y'");
                break;

            case Stage::MovingToY:
                stage_ = Stage::MovingToX2;
                send_robot_command("X'");
                break;

            case Stage::MovingToX2:
                stage_ = Stage::MovingToView2;
                send_robot_command("PICTURE");
                break;

            case Stage::MovingToView2:
                rclcpp::sleep_for(std::chrono::milliseconds(settle_ms_after_arrival_));
                stage_ = Stage::WaitingImage2;
                RCLCPP_INFO(get_logger(), "Robot en pose 2 (Tras secuencia X,Y,X). Capturando imagen 2...");
                break;

            default:
                // En estados Idle, WaitingImage1, WaitingImage2 o Done ignoramos los ARRIVED
                break;
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Bloqueo: Solo procesamos imágenes si la máquina de estados está esperando explícitamente una captura
        if (stage_ != Stage::WaitingImage1 && stage_ != Stage::WaitingImage2) {
            return;
        }

        try {
            const cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            if (stage_ == Stage::WaitingImage1) {
                colors_view_1_ = extract_27_colors_from_mat(
                    frame,
                    circles_config_1_,
                    write_debug_ ? "debug_live_imagen1_circles.jpg" : ""
                );

                // IMPORTANTE: Al terminar la imagen 1, disparamos el primer paso de la nueva secuencia
                stage_ = Stage::MovingToX1;
                RCLCPP_INFO(get_logger(), "Imagen 1 procesada. Iniciando secuencia encadenada: X -> Y -> X -> PICTURE_2");
                send_robot_command("X'"); 
                return;
            }

            if (stage_ == Stage::WaitingImage2) {
                colors_view_2_ = extract_27_colors_from_mat(
                    frame,
                    circles_config_2_,
                    write_debug_ ? "debug_live_imagen2_circles.jpg" : ""
                );

                stage_ = Stage::Done;
                RCLCPP_INFO(get_logger(), "Imagen 2 procesada. Construyendo estado final.");
                const std::string state = build_solver_state_from_current_colors();
                publish_state(state);
                return;
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error procesando imagen live: %s", e.what());
        }
    }
#endif
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeVisionNode>());
    rclcpp::shutdown();
    return 0;
}