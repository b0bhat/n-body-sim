#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
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
    bool massive;
    Color color;
    std::vector<std::pair<double, double>> orbit;

    Body(double x_, double y_, double vx_, double vy_, double mass_, bool mobile_, bool massive_, Color color_) :
            x(x_), y(y_), vx(vx_), vy(vy_), mass(mass_), mobile(mobile_), massive(massive_), color(color_) {}
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


void updateBodies(std::vector<Body>& particles, std::vector<Body*>& massiveParticles, std::vector<Body*>& mobileParticles, double dt) {
    for (auto& particle : mobileParticles) {
        double fx = 0.0, fy = 0.0;
        for (const auto& other : massiveParticles) {
            if (particle != other) { // Comparing pointers
                double partialFx, partialFy;
                calculateForce(*particle, *other, partialFx, partialFy); // Dereferencing pointers to access actual objects
                fx += partialFx;
                fy += partialFy;
            }
        }
        double ax = fx / particle->mass; // Dereferencing pointer
        double ay = fy / particle->mass; // Dereferencing pointer
        particle->vx += ax * dt;
        particle->vy += ay * dt;
        particle->x += particle->vx * dt;
        particle->y += particle->vy * dt;
        particle->orbit.push_back({particle->x, particle->y});
        while (particle->orbit.size() > 750) {
            particle->orbit.erase(particle->orbit.begin());
        }
    }
}


void drawOrbit(const Body& particle) {
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < particle.orbit.size(); ++i) {
        double alpha = 0.8 - std::pow(0.5, static_cast<double>(i) / particle.orbit.size());
        //std::cout << alpha << std::endl;
        glColor4f(particle.color.r, particle.color.g, particle.color.b, alpha);
        glVertex2d(particle.orbit[i].first, particle.orbit[i].second);
    }
    glEnd();

    double trailLength = std::sqrt(particle.vx * particle.vx + particle.vy * particle.vy);
    glBegin(GL_LINE_STRIP);
    size_t startIndex = particle.orbit.size() > trailLength ? particle.orbit.size() - trailLength : 0;
    for (size_t i = startIndex; i < particle.orbit.size(); ++i) {
        glColor4f(particle.color.r, particle.color.g, particle.color.b, 1.0);
        glVertex2d(particle.orbit[i].first, particle.orbit[i].second);
    }
    glEnd();
}

int main() {
    const double dt = 0.005;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> pos_dist(-0.3, -0.300001);
    //std::uniform_real_distribution<double> pos_dist(-0.5, 0.5);
    //std::uniform_real_distribution<double> vel_dist(-3, 3);
    std::uniform_real_distribution<double> color_dist(0.0, 1.0);
    //std::uniform_real_distribution<double> mass_dist(1e1, 1e4);

    std::vector<Body> particles;
    std::vector<Body*> massiveParticles;
    std::vector<Body*> mobileParticles;
    for (int i = 0; i < 1000; ++i) {
        double x = pos_dist(gen);
        double y = pos_dist(gen);
        double vx = 1;
        double vy = 2;
        double mass = 1e2;
        bool mobile = true;
        bool massive = false;
        Color color = Color(color_dist(gen), color_dist(gen), color_dist(gen));
        particles.emplace_back(x, y, vx, vy, mass, mobile, massive, color);
    }
    particles.emplace_back(0.0, 0.15, 0.0, 0.0, 1e6, false, true, Color(1.0, 1.0, 1.0));
    particles.emplace_back(0.1, 0.0, 0.0, 0.0, 1e6, false, true, Color(1.0, 1.0, 1.0));
    particles.emplace_back(-0.1, 0.0, 0.0, 0.0, 1e6, false, true, Color(1.0, 1.0, 1.0));

    for (auto& particle : particles) {
        if (particle.massive) {
            massiveParticles.push_back(&particle);
        }
    }

    for (auto& particle : particles) {
        if (particle.mobile) {
            mobileParticles.push_back(&particle);
        }
    }

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
        updateBodies(particles, massiveParticles, mobileParticles, dt);
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
