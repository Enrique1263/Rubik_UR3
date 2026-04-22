#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp> 
#include <fstream>
#include <vector>
#include <cmath>

using json = nlohmann::json;

class CubeVisionNode : public rclcpp::Node {
public:
    CubeVisionNode() : Node("cube_vision_node") {
        // Cargar círculos desde el JSON (ajusta la ruta a tu archivo)
        load_circles_config("/home/enrique/rubik_ws/src/rubik_viz/src/circles.json");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&CubeVisionNode::image_callback, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_status", 10, std::bind(&CubeVisionNode::status_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Nodo de Visión con lógica original cargado.");
    }

private:
    struct Circle {
        int id;
        double cx, cy, r;
    };
    std::vector<Circle> circles_config_;
    bool ready_to_process_ = false;

    void load_circles_config(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el JSON de círculos en: %s", path.c_str());
            return;
        }
        json data = json::parse(f);
        int idx = 1;
        for (auto& item : data["circles"]) {
            circles_config_.push_back({idx++, item["center"]["x"], item["center"]["y"], item["radius"]});
        }
    }

    // Traducción exacta de classify_color de tu Python
    std::string classify_color(int r8, int g8, int b8) {
        double r = r8 / 255.0;
        double g = g8 / 255.0;
        double b = b8 / 255.0;

        // Convertir RGB a HSV (OpenCV usa rangos distintos, así que lo hacemos manual como en tu colorsys)
        double max_c = std::max({r, g, b});
        double min_c = std::min({r, g, b});
        double delta = max_c - min_c;
        double h = 0;
        if (delta > 0) {
            if (max_c == r) h = 60.0 * fmod(((g - b) / delta), 6.0);
            else if (max_c == g) h = 60.0 * (((b - r) / delta) + 2.0);
            else if (max_c == b) h = 60.0 * (((r - g) / delta) + 4.0);
        }
        if (h < 0) h += 360.0;
        double s = (max_c == 0) ? 0 : (delta / max_c);
        double v = max_c;

        int spread = std::max({r8, g8, b8}) - std::min({r8, g8, b8});

        // Tu lógica de "White"
        if (v >= 0.45 && s <= 0.28 && spread <= 60 && r8 >= 95 && g8 >= 95 && b8 >= 95) return "white";

        // Clasificación por Hue
        if (h < 15 || h >= 345) return "red";
        if (h >= 15 && h < 40) return "orange";
        if (h >= 40 && h < 75) return "yellow";
        if (h >= 75 && h < 170) return "green";
        if (h >= 170 && h < 255) return "blue";

        if (v >= 0.40 && s <= 0.35 && spread <= 70) return "white";

        return "unknown";
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!ready_to_process_) return;

        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            for (const auto& c : circles_config_) {
                // Traducción de pixels_inside_circle (usando inner_ratio 0.55)
                double effective_r = c.r * 0.55;
                long sum_r = 0, sum_g = 0, sum_b = 0;
                int count = 0;

                for (int y = c.cy - effective_r; y <= c.cy + effective_r; ++y) {
                    for (int x = c.cx - effective_r; x <= c.cx + effective_r; ++x) {
                        if (x < 0 || x >= frame.cols || y < 0 || y >= frame.rows) continue;
                        
                        double dist_sq = std::pow(x - c.cx, 2) + std::pow(y - c.cy, 2);
                        if (dist_sq <= std::pow(effective_r, 2)) {
                            cv::Vec3b pixel = frame.at<cv::Vec3b>(y, x);
                            sum_b += pixel[0]; sum_g += pixel[1]; sum_r += pixel[2];
                            count++;
                        }
                    }
                }

                if (count > 0) {
                    std::string color = classify_color(sum_r/count, sum_g/count, sum_b/count);
                    RCLCPP_INFO(this->get_logger(), "Círculo %d: %s (R:%ld G:%ld B:%ld)", 
                                c.id, color.c_str(), sum_r/count, sum_g/count, sum_b/count);
                }
            }

            ready_to_process_ = false;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error bridge: %s", e.what());
        }
    }

    void status_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.find("ARRIVED") != std::string::npos) ready_to_process_ = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeVisionNode>());
    rclcpp::shutdown();
    return 0;
}