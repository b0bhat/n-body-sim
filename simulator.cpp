#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <GLFW/glfw3.h>

struct Color {
    float r, g, b, a;
    Color(float red, float green, float blue, float alpha = 1.0f)
        : r(red), g(green), b(blue), a(alpha) {}
};

struct Body {
    double x, y;
    double vx, vy;
    double mass;
    bool mobile;
    Color color;
    std::vector<std::pair<double, double>> orbit;

    Body(double x_, double y_, double vx_, double vy_, double mass_, bool mobile_, Color color_) :
            x(x_), y(y_), vx(vx_), vy(vy_), mass(mass_), mobile(mobile_), color(color_) {}
};

const double G = 6.67430e-7;

void calculateForce(const Body& p1, const Body& p2, double& fx, double& fy) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double distance = std::max(std::sqrt(dx * dx + dy * dy), 0.1);
    fx = 0.0;
    fy = 0.0;
    if (distance <= 1.5) {
        double force = (G * p1.mass * p2.mass) / (distance * distance);
        fx = force * (dx / distance);
        fy = force * (dy / distance);
    }
}


void updateBodies(std::vector<Body>& particles, double dt) {
    for (auto& particle : particles) {
        double fx = 0.0, fy = 0.0;
        if (particle.mobile) {
            for (const auto& other : particles) {
                if (&particle != &other) {
                    double partialFx, partialFy;
                    calculateForce(particle, other, partialFx, partialFy);
                    fx += partialFx;
                    fy += partialFy;
                }
            }
        }
        double ax = fx / particle.mass;
        double ay = fy / particle.mass;
        particle.vx += ax * dt;
        particle.vy += ay * dt;
        particle.x += particle.vx * dt;
        particle.y += particle.vy * dt;
        particle.orbit.push_back({particle.x, particle.y});
        while (particle.orbit.size() > 1000) {
            particle.orbit.erase(particle.orbit.begin());
        }
    }
}

void drawOrbit(const Body& particle) {
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < particle.orbit.size(); ++i) {
        double alpha = 0.65 - std::pow(0.15, static_cast<double>(i) / particle.orbit.size());
        //std::cout << alpha << std::endl;
        glColor4f(particle.color.r, particle.color.g, particle.color.b, alpha);
        glVertex2d(particle.orbit[i].first, particle.orbit[i].second);
    }
    glEnd();
}

int main() {
    const double dt = 0.005;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> pos_dist(-0.5, 0.5);
    std::uniform_real_distribution<double> vel_dist(-2, 2);
    std::uniform_real_distribution<double> color_dist(0.0, 1.0);
    std::uniform_real_distribution<double> mass_dist(1e1, 1e4);

    std::vector<Body> particles;
    for (int i = 0; i < 1000; ++i) {
        double x = pos_dist(gen);
        double y = pos_dist(gen);
        double vx = vel_dist(gen);
        double vy = vel_dist(gen);
        double mass = mass_dist(gen);
        bool mobile = true;
        Color color = Color(color_dist(gen), color_dist(gen), color_dist(gen));
        particles.emplace_back(x, y, vx, vy, mass, mobile, color);
    }
    particles.emplace_back(0.1, 0.0, 0.0, 0.0, 1e6, false, Color(1.0, 1.0, 1.0));
    particles.emplace_back(-0.1, 0.0, 0.0, 0.0, 1e6, false, Color(1.0, 1.0, 1.0));

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    GLFWwindow* window;
    window = glfwCreateWindow(800, 800, "N-Body Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window" << std::endl;
        return -1;
    }
    glfwMakeContextCurrent(window);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    while (!glfwWindowShouldClose(window)) {
        updateBodies(particles, dt);
        glClear(GL_COLOR_BUFFER_BIT);
        for (const auto& particle : particles) {
            drawOrbit(particle);
        }
        
        for (const auto& particle : particles) {
            glPointSize(std::log(particle.mass)/3);
            glBegin(GL_POINTS);
            glColor3f(particle.color.r, particle.color.g, particle.color.b);
            glVertex2d(particle.x, particle.y);
            glEnd();
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;

}
