#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

namespace cube_vision {

using json = nlohmann::json;

struct Circle {
    int id = 0;
    double cx = 0.0;
    double cy = 0.0;
    double r = 0.0;
};

struct StickerSample {
    int circle_id = 0;
    std::string color;
    int r = 0;
    int g = 0;
    int b = 0;
};

inline std::vector<Circle> load_circles_config(const std::string &path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("No se pudo abrir circles config: " + path);
    }

    json data = json::parse(f);
    std::vector<Circle> circles;

    if (data.is_object() && data.contains("circles")) {
        int idx = 1;
        for (const auto &item : data["circles"]) {
            Circle c;
            c.id = item.value("id", idx);
            c.cx = item["center"].value("x", 0.0);
            c.cy = item["center"].value("y", 0.0);
            c.r = item.value("radius", 0.0);
            circles.push_back(c);
            idx++;
        }
    } else if (data.is_array()) {
        int idx = 1;
        for (const auto &item : data) {
            Circle c;
            c.id = item.value("id", idx);
            c.cx = item.value("cx", item.value("x", 0.0));
            c.cy = item.value("cy", item.value("y", 0.0));
            c.r = item.value("r", item.value("radius", 0.0));
            circles.push_back(c);
            idx++;
        }
    }

    std::sort(circles.begin(), circles.end(), [](const Circle &a, const Circle &b) {
        return a.id < b.id;
    });

    if (circles.size() != 27) {
        throw std::runtime_error("El config de circulos debe tener 27 puntos. Tiene: " + std::to_string(circles.size()));
    }

    return circles;
}

inline std::string nearest_cube_color(int r8, int g8, int b8) {
    struct Ref { const char *name; int r, g, b; };
    const std::array<Ref, 6> refs{{
        {"white",  240, 240, 240},
        {"yellow", 220, 210,  30},
        {"red",    185,  25,  30}, 
        {"orange", 240,  95,  15}, 
        {"green",   35, 155,  60},
        {"blue",    20,  70, 175},
    }};

    double best = 1e18;
    std::string best_name = "unknown";
    for (const auto &ref : refs) {
        const double dr = static_cast<double>(r8 - ref.r);
        const double dg = static_cast<double>(g8 - ref.g);
        const double db = static_cast<double>(b8 - ref.b);
        const double score = dr * dr + dg * dg + db * db;
        if (score < best) {
            best = score;
            best_name = ref.name;
        }
    }
    return best_name;
}

inline std::string classify_color_strict(int r8, int g8, int b8) {
    const double r = r8 / 255.0;
    const double g = g8 / 255.0;
    const double b = b8 / 255.0;

    const double max_c = std::max({r, g, b});
    const double min_c = std::min({r, g, b});
    const double delta = max_c - min_c;

    double h = 0.0;
    if (delta > 0.0) {
        if (max_c == r) {
            h = 60.0 * std::fmod(((g - b) / delta), 6.0);
        } else if (max_c == g) {
            h = 60.0 * (((b - r) / delta) + 2.0);
        } else {
            h = 60.0 * (((r - g) / delta) + 4.0);
        }
    }
    if (h < 0.0) h += 360.0;

    const double s = (max_c == 0.0) ? 0.0 : (delta / max_c);
    const double v = max_c;
    const int spread = std::max({r8, g8, b8}) - std::min({r8, g8, b8});

    // CANDADO ANTI-VERDE PARA EL BLANCO:
    // En un blanco real, el verde nunca debería ganarle al rojo por más de 25 unidades,
    // ni al azul por más de 35. Si hay mucha dominancia de Verde, NO es blanco, es Verde brillante.
    bool looks_like_green = (g8 - r8 > 25) && (g8 - b8 > 30);

    // 1. Filtro estricto para el Blanco (añadiendo la restricción de que no parezca verde)
    if (!looks_like_green) {
        if (v >= 0.45 && s <= 0.30 && spread <= 70 && r8 >= 90 && g8 >= 90 && b8 >= 90) return "white";
        if (v >= 0.40 && s <= 0.38 && spread <= 80) return "white";
    }

    // 2. Zona de conflicto crítico: Rojo vs Naranja (Hue entre 345 y 42 grados)
    if (h < 42.0 || h >= 345.0) {
        double green_red_ratio = static_cast<double>(g8) / (r8 > 0 ? r8 : 1);

        if (h >= 10.0 && h < 42.0) {
            if (green_red_ratio > 0.26 && v > 0.50) {
                return "orange";
            } else {
                return "red";
            }
        }
        if (h >= 345.0 || h < 10.0) return "red";
        return "orange";
    }

    // 3. Resto de colores estables
    if (h >= 42.0 && h < 75.0) return "yellow";
    if (h >= 75.0 && h < 170.0) return "green";
    if (h >= 170.0 && h < 255.0) return "blue";

    return "unknown";
}

inline std::string classify_color(int r8, int g8, int b8) {
    const std::string strict = classify_color_strict(r8, g8, b8);
    if (strict != "unknown") return strict;

    // Fallback práctico: evita abortar cuando una pegatina cae en una zona HSV rara
    // por sombra/reflejo. Si el punto está mal calibrado, el debug lo hará obvio.
    return nearest_cube_color(r8, g8, b8);
}

inline StickerSample sample_circle(const cv::Mat &bgr, const Circle &c, double inner_ratio = 0.55) {
    const double effective_r = c.r * inner_ratio;
    long sum_r = 0, sum_g = 0, sum_b = 0;
    int count = 0;

    const int y0 = static_cast<int>(std::floor(c.cy - effective_r));
    const int y1 = static_cast<int>(std::ceil(c.cy + effective_r));
    const int x0 = static_cast<int>(std::floor(c.cx - effective_r));
    const int x1 = static_cast<int>(std::ceil(c.cx + effective_r));
    const double r2 = effective_r * effective_r;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            if (x < 0 || x >= bgr.cols || y < 0 || y >= bgr.rows) continue;
            const double dx = x - c.cx;
            const double dy = y - c.cy;
            if (dx * dx + dy * dy <= r2) {
                const cv::Vec3b px = bgr.at<cv::Vec3b>(y, x);
                sum_b += px[0];
                sum_g += px[1];
                sum_r += px[2];
                ++count;
            }
        }
    }

    if (count <= 0) {
        return {c.id, "unknown", 0, 0, 0};
    }

    const int rr = static_cast<int>(sum_r / count);
    const int gg = static_cast<int>(sum_g / count);
    const int bb = static_cast<int>(sum_b / count);
    return {c.id, classify_color(rr, gg, bb), rr, gg, bb};
}

inline std::vector<std::string> extract_27_colors_from_image(
    const std::string &image_path,
    const std::vector<Circle> &circles,
    const std::string &debug_output_path = "") {

    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        throw std::runtime_error("No se pudo leer la imagen: " + image_path);
    }

    std::vector<std::string> colors;
    colors.reserve(circles.size());

    cv::Mat debug;
    if (!debug_output_path.empty()) image.copyTo(debug);

    std::ofstream report;
    if (!debug_output_path.empty()) {
        const std::string report_path = debug_output_path + ".csv";
        report.open(report_path);
        report << "id,cx,cy,r,avg_r,avg_g,avg_b,color,face_slot,sticker_slot,is_center\n";
    }

    for (const auto &c : circles) {
        const StickerSample sample = sample_circle(image, c);
        colors.push_back(sample.color);

        const int zero_based = static_cast<int>(colors.size()) - 1;
        const int face_slot = zero_based / 9;
        const int sticker_slot = zero_based % 9;
        const bool is_center = sticker_slot == 4;
        if (report.is_open()) {
            report << sample.circle_id << "," << c.cx << "," << c.cy << "," << c.r << ","
                   << sample.r << "," << sample.g << "," << sample.b << ","
                   << sample.color << "," << face_slot << "," << sticker_slot << ","
                   << (is_center ? "yes" : "no") << "\n";
        }

        if (!debug_output_path.empty()) {
            cv::Scalar draw_color(180, 180, 180);
            if (sample.color == "white") draw_color = cv::Scalar(255, 255, 255);
            else if (sample.color == "yellow") draw_color = cv::Scalar(0, 255, 255);
            else if (sample.color == "red") draw_color = cv::Scalar(0, 0, 255);
            else if (sample.color == "orange") draw_color = cv::Scalar(0, 165, 255);
            else if (sample.color == "green") draw_color = cv::Scalar(0, 255, 0);
            else if (sample.color == "blue") draw_color = cv::Scalar(255, 0, 0);
            cv::circle(debug, cv::Point(static_cast<int>(c.cx), static_cast<int>(c.cy)), static_cast<int>(c.r), draw_color, 2);
            cv::putText(debug, std::to_string(c.id) + ":" + sample.color.substr(0, 1),
                        cv::Point(static_cast<int>(c.cx - c.r), static_cast<int>(c.cy - c.r)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1);
        }
    }

    if (!debug_output_path.empty()) {
        cv::imwrite(debug_output_path, debug);
    }

    return colors;
}

inline std::vector<std::vector<std::string>> split_faces(const std::vector<std::string> &colors) {
    if (colors.size() != 27) {
        throw std::runtime_error("Cada imagen debe producir 27 colores, no " + std::to_string(colors.size()));
    }
    return {
        std::vector<std::string>(colors.begin(), colors.begin() + 9),
        std::vector<std::string>(colors.begin() + 9, colors.begin() + 18),
        std::vector<std::string>(colors.begin() + 18, colors.begin() + 27)
    };
}

inline char color_to_face_char(const std::string &color) {
    if (color == "white") return 'U';
    if (color == "yellow") return 'D';
    if (color == "red") return 'F';
    if (color == "orange") return 'B';
    if (color == "green") return 'L';
    if (color == "blue") return 'R';
    throw std::runtime_error("Color desconocido al convertir a solver: " + color);
}

inline char center_color_to_face_name(const std::string &center_color) {
    return color_to_face_char(center_color);
}

inline std::map<char, std::vector<std::string>> build_cube_from_two_views(
    const std::vector<std::string> &colors_1,
    const std::vector<std::string> &colors_2) {

    auto faces_1 = split_faces(colors_1);
    auto faces_2 = split_faces(colors_2);

    std::map<char, std::vector<std::string>> cube;
    std::set<char> seen;

    for (const auto &face : {faces_1[0], faces_1[1], faces_1[2], faces_2[0], faces_2[1], faces_2[2]}) {
        const std::string &center = face[4];
        if (center == "unknown") {
            throw std::runtime_error("Centro unknown en una de las 6 caras. Mira debug_imagen*_circles.jpg y sus CSV: probablemente el circles.json no cae encima del centro de esa cara.");
        }
        const char face_name = center_color_to_face_name(center);
        if (seen.count(face_name)) {
            throw std::runtime_error(std::string("Cara repetida por centro de color: ") + face_name + " (" + center + ")");
        }
        seen.insert(face_name);
        cube[face_name] = face;
    }

    for (char f : std::string("URFDLB")) {
        if (!cube.count(f)) {
            throw std::runtime_error(std::string("Falta la cara ") + f + ". Revisa calibracion/orden de los 27 puntos.");
        }
    }

    return cube;
}

inline std::string cube_to_solver_string(const std::map<char, std::vector<std::string>> &cube) {
    const std::string order = "URFDLB";
    std::string out;
    out.reserve(54);

    for (char face : order) {
        const auto it = cube.find(face);
        if (it == cube.end() || it->second.size() != 9) {
            throw std::runtime_error(std::string("Cara incompleta: ") + face);
        }
        for (const auto &color : it->second) {
            out.push_back(color_to_face_char(color));
        }
    }
    return out;
}

inline bool validate_solver_string(const std::string &s, std::string *error = nullptr) {
    if (s.size() != 54) {
        if (error) *error = "Longitud invalida: " + std::to_string(s.size()) + ", esperaba 54";
        return false;
    }

    std::map<char, int> count;
    for (char c : s) count[c]++;

    for (char c : std::string("URFDLB")) {
        if (count[c] != 9) {
            if (error) *error = std::string("Conteo invalido para ") + c + ": " + std::to_string(count[c]) + ", esperaba 9";
            return false;
        }
    }
    return true;
}

inline void print_faces(const std::string &s, std::ostream &os = std::cout) {
    const std::string order = "URFDLB";
    for (int i = 0; i < 6; ++i) {
        const std::string block = s.substr(i * 9, 9);
        os << "\n" << order[i] << ":\n";
        os << block.substr(0, 3) << "\n";
        os << block.substr(3, 3) << "\n";
        os << block.substr(6, 3) << "\n";
    }
}

inline std::string state_from_two_images(
    const std::string &image_1_path,
    const std::string &image_2_path,
    const std::string &circles_config_1_path,
    const std::string &circles_config_2_path = "",
    bool write_debug = false) {

    const auto circles_1 = load_circles_config(circles_config_1_path);
    const auto circles_2 = load_circles_config(circles_config_2_path.empty() ? circles_config_1_path : circles_config_2_path);

    const auto colors_1 = extract_27_colors_from_image(
        image_1_path, circles_1, write_debug ? "debug_imagen1_circles.jpg" : "");
    const auto colors_2 = extract_27_colors_from_image(
        image_2_path, circles_2, write_debug ? "debug_imagen2_circles.jpg" : "");

    const auto cube = build_cube_from_two_views(colors_1, colors_2);
    const std::string state = cube_to_solver_string(cube);

    std::string error;
    if (!validate_solver_string(state, &error)) {
        throw std::runtime_error(error);
    }

    return state;
}

} // namespace cube_vision
