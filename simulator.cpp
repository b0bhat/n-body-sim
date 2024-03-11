#include "simulator.h"

std::vector<CollisionPoint> collisionPoints;

bool isSpacePressed() {
    HWND foregroundWindow = GetForegroundWindow();
    DWORD currentProcessId;
    GetWindowThreadProcessId(foregroundWindow, &currentProcessId);
    DWORD thisProcessId = GetCurrentProcessId();
    if (currentProcessId != thisProcessId) {
        return false;
    }
    return GetAsyncKeyState(VK_SPACE) & 0x8000;
}

void restartExecutable(GLFWwindow* window) {
    std::cout << "Restarting...\n";
    run(window);
    return;
}

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

void calculateForce(const Body& p1, const Body& p2, double& fx, double& fy) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double distance = std::max(std::sqrt(dx * dx + dy * dy), 0.1);
    fx = 0.0;
    fy = 0.0;
    if (distance <= 1.0) {
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
        double distanceSquared = dx * dx + dy * dy;
        if (distanceSquared < 0.001f * 0.001f) {
            if (other->massive && !other->mobile) {
                particle->alive = false;
                return;
            } else {
                double totalMass = particle->mass + other->mass;
                particle->vx = (particle->mass * particle->vx + other->mass * other->vx) / totalMass;
                particle->vy = (particle->mass * particle->vy + other->mass * other->vy) / totalMass;
                particle->mass = totalMass; 
                other->alive = false;
                collided = true;
                CollisionPoint point;
                point.x = particle->x;
                point.y = particle->y;
                point.mass = totalMass;
                point.timestamp = std::chrono::steady_clock::now();
                collisionPoints.push_back(point);
                break;
            }
        }
    }
    if (collided) {
        return;
    }
}

void updateBodies(std::vector<Body>& particles, std::vector<Body*>& massiveParticles, std::vector<Body*>& mobileParticles, double dt) {
    const size_t maxOrbitSize = 1000;
    for (auto& particle : mobileParticles) {
        if (particle->alive && particle->collision) {
            handleCollisions(particle, massiveParticles, mobileParticles);
        }
        if (!particle->alive) {
            continue;
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
        while (particle->orbit.size() > maxOrbitSize) {
            particle->orbit.erase(particle->orbit.begin());
        }
        if (std::sqrt(std::pow(particle->x, 2) + std::pow(particle->y, 2)) > 1.5) {
            particle->alive = false;
        }
    }
}


void drawCollisionDots() {
    auto currentTime = std::chrono::steady_clock::now();
    for (auto it = collisionPoints.begin(); it != collisionPoints.end();) {
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - it->timestamp).count();
        if (elapsedTime >= 500) {
            it = collisionPoints.erase(it);
        } else {
            float alpha = std::exp(-elapsedTime / 200.0f);
            glPointSize(std::log(it->mass) * alpha);
            glBegin(GL_POINTS);
            glColor4f(1.0f, 1.0f, 1.0f, alpha);
            glVertex2f(it->x, it->y);
            ++it;
            glEnd();
        }
    }
}

void drawOrbit(const Body& particle, float trailAlpha) {
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < particle.orbit.size(); ++i) {
        double alpha = particle.currentAlpha - std::pow(trailAlpha, static_cast<double>(i) / particle.orbit.size());
        //std::cout << alpha << std::endl;
        glColor4f(particle.color.r, particle.color.g, particle.color.b, alpha);
        glVertex2d(particle.orbit[i].first, particle.orbit[i].second);
    }
    glEnd();
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
}

void computePosVel(
    double& x, double& y, double& vx, double& vy, 
    double minVel, double maxVel, double minRadius, double maxRadius,
    std::mt19937& gen) {
    std::uniform_real_distribution<double> vel_dist(minVel, maxVel);
    std::uniform_real_distribution<double> radius_dist(minRadius, maxRadius);
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * pi);
    double radius = radius_dist(gen);
    double angle = angle_dist(gen);

    x = radius * cos(angle);
    y = radius * sin(angle);
    double tangent_angle = atan2(y, x) + pi / 2;
    std::uniform_real_distribution<double> velocity_angle_dist(tangent_angle - pi / 5, tangent_angle + pi / 5);
    double velocity_angle = velocity_angle_dist(gen);
    double vel_amount = vel_dist(gen);
    std::uniform_int_distribution<int> sign_gen(0, 1);
    int sign = sign_gen(gen) * 2 - 1;
    vx = sign * cos(velocity_angle) * vel_amount;
    vy = sign * sin(velocity_angle) * vel_amount;
}

int run(GLFWwindow* window) {
    std::ifstream inputFile("settings.txt");
    if (!inputFile.is_open()) {
        std::cerr << "Error opening settings file." << std::endl;
        return 1;
    }
    double dt;
    int singleStart, numBodiesGenerator, collidingBodies, numMainBodies;
    double singleStartDelta;
    float trailAlpha;
    double minMass, maxMass, minHueSize, maxHueSize, minVel, maxVel, minRadius, maxRadius;
    inputFile >> dt >> singleStart >> numBodiesGenerator >> collidingBodies >> numMainBodies
        >> singleStartDelta >> trailAlpha >> minMass >> maxMass >> minHueSize >> maxHueSize
        >> minVel >> maxVel >> minRadius >> maxRadius;
    inputFile.close();
    std::cout << "dt: " << dt << std::endl;
    std::cout << "singleStart: " << singleStart << std::endl;
    std::cout << "numBodiesGenerator: " << numBodiesGenerator << std::endl;
    std::cout << "collidingBodies: " << collidingBodies << std::endl;
    std::cout << "numMainBodies: " << numMainBodies << std::endl;
    std::cout << "singleStartDelta: " << singleStartDelta << std::endl;
    std::cout << "trailAlpha: " << trailAlpha << std::endl;
    std::cout << "minMass: " << minMass << std::endl;
    std::cout << "maxMass: " << maxMass << std::endl;
    std::cout << "minHueSize: " << minHueSize << std::endl;
    std::cout << "maxHueSize: " << maxHueSize << std::endl;
    std::cout << "minVel: " << minVel << std::endl;
    std::cout << "maxVel: " << maxVel << std::endl;
    std::cout << "minRadius: " << minRadius << std::endl;
    std::cout << "maxRadius: " << maxRadius << std::endl;

    std::cout << "Press SPACE to restart." << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> mass_dist(minMass, maxMass);
    std::uniform_real_distribution<double> hue_size_dist(minHueSize, maxHueSize);

    std::uniform_real_distribution<double> color_dist(0.3, 1.0);
    double x_s, y_s, vx_s, vy_s;
    computePosVel(x_s, y_s, vx_s, vy_s, minVel, maxVel, minRadius, maxRadius, gen);
    std::uniform_real_distribution<double> x_dist(x_s, x_s + singleStartDelta);
    std::uniform_real_distribution<double> y_dist(y_s, y_s + singleStartDelta);

    double colorStart = color_dist(gen);
    double colorEnd = colorStart + hue_size_dist(gen);
    std::uniform_real_distribution<double> hue_range_dist(colorStart, colorEnd);

    std::vector<Body> particles;
    std::vector<Body*> massiveParticles;
    std::vector<Body*> mobileParticles;

    collisionPoints.clear();

    for (int i = 0; i < numBodiesGenerator; ++i) {
        
        double x, y, vx, vy;
        computePosVel(x, y, vx, vy, minVel, maxVel, minRadius, maxRadius, gen);
        double mass = mass_dist(gen);
        bool mobile = true;
        bool massive = true;
        bool collision = false;
        if (collidingBodies == 1) {
            collision = true;
        } 
        bool alive = true;
        float hue = hue_range_dist(gen);
        float lightness = color_dist(gen);
        if (singleStart == 1) {
            x = x_dist(gen);
            y = y_dist(gen);
            vx = vx_s/2;
            vy = vy_s/2;
            mass = 1e2;
            massive = false;
            float hue = static_cast<float>((x - x_s) / singleStartDelta);
            hue = fmod(hue + 1.0f, 1.0f);
            hue = colorStart + hue * (colorEnd - colorStart);
        }
        float r, g, b;
        hsvToRgb(hue, 1.0f, lightness, r, g, b);
        Color color = Color(r,g,b);
        particles.emplace_back(x, y, vx, vy, mass, mobile, massive, collision, alive, color);
    }
    if (numMainBodies == 1) {
        particles.emplace_back(0.0, 0.0, 0.0, 0.0, 1e7, false, true, false, true, Color(1.0, 1.0, 1.0));
    } else if (numMainBodies >= 2) {
        particles.emplace_back(-0.1, 0.0, 0.0, 0.0, 1e6, false, true, false, true, Color(1.0, 1.0, 1.0));
        particles.emplace_back(0.1, 0.0, 0.0, 0.0, 1e6, false, true, false, true, Color(1.0, 1.0, 1.0));
    }
    if (numMainBodies == 3) {
        particles.emplace_back(0.0, 0.15, 0.0, 0.0, 1e6, false, true, false, true, Color(1.0, 1.0, 1.0));
    }

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

    auto start_time = std::chrono::steady_clock::now();
    while (!glfwWindowShouldClose(window)) {
        if (isSpacePressed()) {
            return 0;
        }
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::minutes>(current_time - start_time).count();
        if (elapsed_time >= 3) {
            return 0;
        }
        glClear(GL_COLOR_BUFFER_BIT);
        drawCollisionDots();
        for (const auto& particle : particles) {
            drawOrbit(particle, trailAlpha);
            if (particle.alive) {
                glPointSize(std::log(particle.mass)/3);
                glBegin(GL_POINTS);
                glColor3f(particle.color.r, particle.color.g, particle.color.b);
                glVertex2d(particle.x, particle.y);
                glEnd();
            }
        }
        updateBodies(particles, massiveParticles, mobileParticles, dt);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    std::cout << "Closing" << std::endl;
    return 1;
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    GLFWwindow* window;
    window = glfwCreateWindow(500, 500, "N-Body Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window" << std::endl;
        return -1;
    }
    glfwMakeContextCurrent(window);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    while (true) {
        int result = run(window);
        if (result != 0) {
            return 0;
        }
    }
        
    std::cout << "Closing" << std::endl;
    return 0;
}
