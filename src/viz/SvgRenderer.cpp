#include "viz/SvgRenderer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace af {

namespace {

struct Projector {
    double scale{1.0};
    double offset_x{0.0};
    double offset_y{0.0};

    static constexpr double kCos30 = 0.8660254037844386;
    static constexpr double kSin30 = 0.5;

    [[nodiscard]] Vec3 project(const Vec3& p) const {
        const double px = (p.x - p.y) * kCos30;
        const double py = (p.x + p.y) * kSin30 - p.z;
        return Vec3{offset_x + px * scale, offset_y + py * scale, 0.0};
    }
};

std::string fmt(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
}

}  // namespace

std::string render_svg(const ArmModel& model, const std::vector<std::vector<double>>& poses,
                       const std::vector<Vec3>& path, const SvgOptions& options) {
    std::vector<Vec3> points;
    points.reserve(poses.size() * (model.dof() + 1) + path.size());
    for (const auto& q : poses) {
        for (const auto& frame : model.link_frames(q)) points.push_back(frame.translation());
    }
    for (const auto& p : path) points.push_back(p);
    if (points.empty()) points.push_back(Vec3{});

    double min_x = 1e9;
    double max_x = -1e9;
    double min_y = 1e9;
    double max_y = -1e9;
    for (const auto& p : points) {
        const double px = (p.x - p.y) * Projector::kCos30;
        const double py = (p.x + p.y) * Projector::kSin30 - p.z;
        min_x = std::min(min_x, px);
        max_x = std::max(max_x, px);
        min_y = std::min(min_y, py);
        max_y = std::max(max_y, py);
    }

    const double span_x = std::max(1e-3, max_x - min_x);
    const double span_y = std::max(1e-3, max_y - min_y);
    const double usable_w = static_cast<double>(options.width) - 2.0 * options.padding;
    const double usable_h = static_cast<double>(options.height) - 2.0 * options.padding;

    Projector projector;
    projector.scale = std::min(usable_w / span_x, usable_h / span_y);
    projector.offset_x = options.padding + (usable_w - span_x * projector.scale) * 0.5 -
                         min_x * projector.scale;
    projector.offset_y = options.padding + (usable_h - span_y * projector.scale) * 0.5 -
                         min_y * projector.scale;

    std::ostringstream svg;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << options.width << "\" height=\""
        << options.height << "\" viewBox=\"0 0 " << options.width << " " << options.height
        << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#0f1115\"/>\n";

    if (options.draw_grid) {
        svg << "<g stroke=\"#1f2530\" stroke-width=\"1\">\n";
        for (int i = -8; i <= 8; ++i) {
            const double value = static_cast<double>(i) * 0.1;
            const auto a = projector.project(Vec3{value, -0.8, 0.0});
            const auto b = projector.project(Vec3{value, 0.8, 0.0});
            svg << "<line x1=\"" << fmt(a.x) << "\" y1=\"" << fmt(a.y) << "\" x2=\"" << fmt(b.x)
                << "\" y2=\"" << fmt(b.y) << "\"/>\n";
            const auto c = projector.project(Vec3{-0.8, value, 0.0});
            const auto d = projector.project(Vec3{0.8, value, 0.0});
            svg << "<line x1=\"" << fmt(c.x) << "\" y1=\"" << fmt(c.y) << "\" x2=\"" << fmt(d.x)
                << "\" y2=\"" << fmt(d.y) << "\"/>\n";
        }
        svg << "</g>\n";
    }

    if (options.draw_path && path.size() > 1) {
        svg << "<polyline fill=\"none\" stroke=\"#38bdf8\" stroke-width=\"2.5\" opacity=\"0.9\" points=\"";
        for (const auto& p : path) {
            const auto projected = projector.project(p);
            svg << fmt(projected.x) << "," << fmt(projected.y) << " ";
        }
        svg << "\"/>\n";
    }

    // Draw every sampled pose faintly, then the last one on top.
    for (std::size_t pose_index = 0; pose_index < poses.size(); ++pose_index) {
        const bool last = pose_index + 1 == poses.size();
        const auto frames = model.link_frames(poses[pose_index]);
        svg << "<g stroke=\"" << (last ? "#f97316" : "#334155") << "\" stroke-width=\""
            << (last ? 7.0 : 2.0) << "\" stroke-linecap=\"round\" "
            << "opacity=\"" << (last ? 1.0 : 0.45) << "\">\n";
        for (std::size_t i = 0; i + 1 < frames.size(); ++i) {
            const auto a = projector.project(frames[i].translation());
            const auto b = projector.project(frames[i + 1].translation());
            svg << "<line x1=\"" << fmt(a.x) << "\" y1=\"" << fmt(a.y) << "\" x2=\"" << fmt(b.x)
                << "\" y2=\"" << fmt(b.y) << "\"/>\n";
        }
        svg << "</g>\n";
        if (last) {
            for (const auto& frame : frames) {
                const auto p = projector.project(frame.translation());
                svg << "<circle cx=\"" << fmt(p.x) << "\" cy=\"" << fmt(p.y)
                    << "\" r=\"5\" fill=\"#f8fafc\" stroke=\"#f97316\" stroke-width=\"2\"/>\n";
            }
        }
    }

    svg << "<text x=\"24\" y=\"36\" fill=\"#94a3b8\" font-family=\"monospace\" font-size=\"16\">"
        << model.name() << " | poses: " << poses.size() << " | c++ kinematics demo</text>\n";
    svg << "</svg>\n";
    return svg.str();
}

}  // namespace af
