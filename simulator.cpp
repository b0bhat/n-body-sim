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
    bool collision;
    bool alive;
    Color color;
    std::vector<std::pair<double, double>> orbit;
    mutable double currentAlpha = 0.8;

    Body(double x_, double y_, double vx_, double vy_, double mass_, bool mobile_, bool massive_, bool collision_, bool alive_, Color color_) :
            x(x_), y(y_), vx(vx_), vy(vy_), mass(mass_), mobile(mobile_), massive(massive_), collision(collision_), alive(alive_), color(color_) {}
};

void hsvToRgb(float h, float s, float v, float &r, float &g, float &b) {
    int i = floor(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
}

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

void handleCollisions(Body* particle, std::vector<Body*>& massiveParticles, std::vector<Body*>& mobileParticles) {
    bool collided = false;
    for (size_t j = 0; j < mobileParticles.size(); ++j) {
        Body* other = mobileParticles[j];
        if (particle == other || !other->alive) {
            continue;
        }
        double dx = particle->x - other->x;
        double dy = particle->y - other->y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < 0.001f) {
            particle->mass += other->mass;
            particle->vx = (particle->mass * particle->vx + other->mass * other->vx) / (particle->mass + other->mass);
            particle->vy = (particle->mass * particle->vy + other->mass * other->vy) / (particle->mass + other->mass);
            other->alive = false;
            collided = true;
            break;
        }
    }
    if (collided) {
        return;
    }
}

void updateBodies(std::vector<Body>& particles, std::vector<Body*>& massiveParticles, std::vector<Body*>& mobileParticles, double dt) {
    for (auto& particle : mobileParticles) {
        if (!particle->alive) {
            continue;
        }
        //std::cout << particle->alive << std::endl;
        if (particle->collision) {
            handleCollisions(particle, massiveParticles, mobileParticles);
        }
        double fx = 0.0, fy = 0.0;
        for (const auto& other : massiveParticles) {
            if (particle != other && other->alive) {
                double partialFx, partialFy;
                calculateForce(*particle, *other, partialFx, partialFy);
                fx += partialFx;
                fy += partialFy;
            }
        }
        double ax = fx / particle->mass;
        double ay = fy / particle->mass;
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
    if (particle.alive) {
        double trailLength = std::sqrt(particle.vx * particle.vx + particle.vy * particle.vy);
        glBegin(GL_LINE_STRIP);
        size_t startIndex = particle.orbit.size() > trailLength ? particle.orbit.size() - trailLength : 0;
        for (size_t i = startIndex; i < particle.orbit.size(); ++i) {
            glColor4f(particle.color.r, particle.color.g, particle.color.b, 1.0);
            glVertex2d(particle.orbit[i].first, particle.orbit[i].second);
        }
        glEnd();
    } else {
        particle.currentAlpha = std::max(particle.currentAlpha - 0.01, 0.0);
    }
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < particle.orbit.size(); ++i) {
        double alpha = particle.currentAlpha - std::pow(0.4, static_cast<double>(i) / particle.orbit.size());
        //std::cout << alpha << std::endl;
        glColor4f(particle.color.r, particle.color.g, particle.color.b, alpha);
        glVertex2d(particle.orbit[i].first, particle.orbit[i].second);
    }
    glEnd();
}

int main() {
    const double dt = 0.001;
    bool singleStart = true;
    const double numBodiesGenerator = 500;
    double singleStartDelta = 0.00001;

    std::random_device rd;
    std::mt19937 gen(rd());
    //std::uniform_int_distribution<> sign_dist(0, 1);
    //std::uniform_real_distribution<double> pos_dist(-0.4, 0.4);
    std::uniform_real_distribution<double> vel_dist(-3, 3);
    std::uniform_real_distribution<double> mass_dist(1e1, 1e4);
    std::uniform_real_distribution<double> color_dist(0.0, 1.0);

    std::uniform_real_distribution<double> hue_size_dist(0.3, 0.5);

    std::uniform_real_distribution<double> radius_dist(0.4, 0.5);
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * std::acos(-1.0));
    double radius = radius_dist(gen);
    double angle = angle_dist(gen);
    double x_s = radius * cos(angle);
    double y_s = radius * sin(angle);

    double colorStart = color_dist(gen);
    double colorEnd = colorStart + hue_size_dist(gen);
    std::uniform_real_distribution<double> hue_range_dist(colorStart, colorEnd);

    double vx_s = vel_dist(gen)/1.5;
    double vy_s = vel_dist(gen)/1.5;
    std::uniform_real_distribution<double> x_dist(x_s, x_s + singleStartDelta);
    std::uniform_real_distribution<double> y_dist(y_s, y_s + singleStartDelta);

    std::vector<Body> particles;
    std::vector<Body*> massiveParticles;
    std::vector<Body*> mobileParticles;
    for (int i = 0; i < numBodiesGenerator; ++i) {
        radius = radius_dist(gen);
        angle = angle_dist(gen);
        double x = radius * cos(angle);
        double y = radius * sin(angle);
        double vx = vel_dist(gen);
        double vy = vel_dist(gen);
        double mass = mass_dist(gen);
        bool mobile = true;
        bool massive = true;
        bool collision = false;
        bool alive = true;
        float hue = hue_range_dist(gen);
        float lightness = color_dist(gen);
        // Color color = Color(color_dist(gen), color_dist(gen), color_dist(gen));
        if (singleStart) {
            x = x_dist(gen);
            y = y_dist(gen);
            vx = vx_s;
            vy = vy_s;
            mass = 1e2;
            massive = false;
            //float hue = hue_range_dist(gen);
            float hue = static_cast<float>((x - x_s) / singleStartDelta);
            hue = fmod(hue + 1.0f, 1.0f);
            hue = colorStart + hue * (colorEnd - colorStart);
        }
        lightness = std::min(lightness+0.1, 1.0);
        float r, g, b;
        hsvToRgb(hue, 1.0f, lightness, r, g, b);
        Color color = Color(r,g,b);
        particles.emplace_back(x, y, vx, vy, mass, mobile, massive, collision, alive, color);
    }
    //particles.emplace_back(0.0, 0.15, 0.0, 0.0, 1e7, false, true, true, false, Color(1.0, 1.0, 1.0));
    particles.emplace_back(0.0, 0.15, 0.0, 0.0, 1e6, false, true, false, true, Color(1.0, 1.0, 1.0));
    particles.emplace_back(0.1, 0.0, 0.0, 0.0, 1e6, false, true, false, true, Color(1.0, 1.0, 1.0));
    particles.emplace_back(-0.1, 0.0, 0.0, 0.0, 1e6, false, true, false, true, Color(1.0, 1.0, 1.0));

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
            if (particle.alive) {
                glPointSize(std::log(particle.mass)/3);
                glBegin(GL_POINTS);
                glColor3f(particle.color.r, particle.color.g, particle.color.b);
                glVertex2d(particle.x, particle.y);
                glEnd();
            }
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;

}
