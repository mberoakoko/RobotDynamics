//
// Created by mbero-akoko on 6/29/26.
//

#ifndef ROBOTDYNAMICS_CORE_RENDERING_DEMO_HPP
#define ROBOTDYNAMICS_CORE_RENDERING_DEMO_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Geometry>
#include <vector>
#include <functional>
#include <iostream>
#include <string>
#include <ranges>

namespace dynamics::ather::viz {

    auto inline draw_coordinate_frame(const Eigen::Isometry3d& T_w_f, float scale = 1.0f)-> void {
        Eigen::Matrix4d m = T_w_f.matrix();
        glMultMatrixd(m.data());

        glBegin(GL_LINES);
        // X axis - Red
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(scale, 0.0f, 0.0f);
        // Y axis - Green
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, scale, 0.0f);
        // Z axis - Blue
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, scale);
        glEnd();

        glPopMatrix();
    }


    inline auto draw_grid(float size = 10.0f, int divisions = 10) -> void{
        glBegin(GL_LINES);
        // Set grid line color (e.g., light gray)
        glColor3f(0.6f, 0.6f, 0.6f);

        for(float i = -size; i <= size; i += divisions) {
            // Lines parallel to the Y-axis
            glVertex3f(i, -size, 0);
            glVertex3f(i, size, 0);

            // Lines parallel to the X-axis
            glVertex3f(-size, i, 0);
            glVertex3f(size, i, 0);
        }
        glEnd();
    }

    struct Frame {
        Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
        float scale{1.0f};
        std::string label{"frame"};
        // Future: color, primitive type, custom draw strategy via std::function, etc.
    };

    // Concept for anything that can be drawn (zero-cost abstraction)
    template <typename T>
    concept Renderable = requires(const T& t, const pangolin::OpenGlRenderState& cam) {
        { t.draw(cam) } -> std::same_as<void>;   // or we can relax this
    };

    template <std::ranges::range R>
    auto  render_scene(const R& scene, const pangolin::OpenGlRenderState& cam) -> void {
        for (const auto& item : scene) {
            if constexpr (requires { item.pose; }) {
                // Our Frame struct
                draw_coordinate_frame(item.pose, item.scale);
            } else {
                // Future: other renderables
                item.draw(cam);
            }
        }
    }

    inline auto mock_kinematics_update(double t, const Eigen::Isometry3d& base_pose) -> Eigen::Isometry3d {
        // Simple orbiting + rotating "robot" in space
        Eigen::AngleAxisd rot(t * 0.8, Eigen::Vector3d::UnitZ());
        Eigen::Translation3d trans(
            3.0 * std::sin(t * 0.5),
            2.0 * std::cos(t * 0.3),
            1.5 + 0.5 * std::sin(t * 1.2)
        );
        return base_pose * Eigen::Isometry3d(trans) * Eigen::Isometry3d(rot);
    }


    namespace functionality_test{
        inline auto test_renderer() {
            pangolin::CreateWindowAndBind("Aether - Core Rendering Demo", 1280, 720);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            // Camera setup (good starting view for space robotics)
            pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1280, 720, 420, 420, 640, 360, 0.1, 1000),
                pangolin::ModelViewLookAt(8, -8, 6, 0, 0, 0, pangolin::AxisZ)
            );

            pangolin::Handler3D handler(s_cam);
            pangolin::View &d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f / 720.0f)
                    .SetHandler(&handler);

            // Runtime tweakable variables (Pangolin functional UI)
            pangolin::Var<bool> show_grid("ui.Show Grid", true, true);
            pangolin::Var<float> frame_scale("ui.Frame Scale", 1.0f, 0.1f, 5.0f);
            pangolin::Var<bool> animate("ui.Animate (Mock Kinematics)", true, true);

            // === Scene Data (OO ownership) ===
            std::vector<Frame> scene;
            Frame world_frame;
            world_frame.label = "world";
            world_frame.scale = 2.0f;
            scene.push_back(world_frame);

            Frame robot_frame;
            robot_frame.label = "robot_base";
            robot_frame.pose = Eigen::Isometry3d::Identity();
            scene.push_back(robot_frame);

            // Base pose for the "robot" (could come from URDF later)
            Eigen::Isometry3d robot_base = Eigen::Isometry3d::Identity();
            robot_base.translate(Eigen::Vector3d(0, 0, 0));

            std::cout << "Aether Core Rendering Demo started.\n"
                    << "Controls: Left-drag orbit, Right-drag pan, Scroll zoom, 'r' reset view\n";

            // === Main Render Loop ===
            while (!pangolin::ShouldQuit()) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                // === FUNCTIONAL UPDATE (mock kinematics) ===
                static double t = 0.0;
                if (animate) {
                    t += 0.016; // ~60 FPS
                    scene[1].pose = mock_kinematics_update(t, robot_base);
                    scene[1].scale = frame_scale;
                }

                d_cam.Activate(s_cam);

                // === RENDERING (mix of functional + OO data) ===
                if (show_grid) {
                    draw_grid(20.0f, 20);
                }

                // Functional scene rendering
                render_scene(scene, s_cam);

                // Simple text overlay (functional)
                glColor3f(1.0f, 1.0f, 1.0f);
                // pangolin::GlText txt = pangolin::GlFont::Text();
                // txt.DrawWindow(10, 30);

                pangolin::FinishFrame();
    }
        }

    }


}


#endif //ROBOTDYNAMICS_CORE_RENDERING_DEMO_HPP
