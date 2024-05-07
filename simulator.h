#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <windows.h>
#include <GLFW/glfw3.h>

const double pi = std::acos(-1.0);
const double G = 6.67430e-7;
const int MAX_BODIES_PER_NODE = 8;

struct Color {
    float r, g, b, a;
    Color(float red, float green, float blue, float alpha = 1.0f)
        : r(red), g(green), b(blue), a(alpha) {}
};

struct Body {
    float x, y;
    float vx, vy;
    double mass;
    bool mobile;
    bool massive;
    bool collision;
    bool alive;
    Color color;
    std::vector<std::pair<float, float>> orbit;
    mutable double currentAlpha = 0.8;

    Body(float x_, float y_, float vx_, float vy_, double mass_, bool mobile_, bool massive_, bool collision_, bool alive_, Color color_) :
            x(x_), y(y_), vx(vx_), vy(vy_), mass(mass_), mobile(mobile_), massive(massive_), collision(collision_), alive(alive_), color(color_) {}
};

struct Quadtree {
    float x, y;
    float width, height;
    float centerOfMassX, centerOfMassY;
    float totalMass;
    std::vector<Body*> bodies;
    bool divided = false;
    Quadtree* nw = nullptr;
    Quadtree* ne = nullptr;
    Quadtree* sw = nullptr;
    Quadtree* se = nullptr;

    Quadtree(float x_, float y_, float w_, float h_) : x(x_), y(y_), width(w_), height(h_), 
                                                        centerOfMassX(x_ + w_ / 2.0f), 
                                                        centerOfMassY(y_ + h_ / 2.0f), 
                                                        totalMass(0.0f) {}

    ~Quadtree() {
    }

    void insert(Body* p) {
        if (!contains(p)) return;

        bodies.push_back(p);

        if (!divided && bodies.size() > MAX_BODIES_PER_NODE) {
            subdivide();
            for (auto& body : bodies) {
                if (nw->contains(body)) nw->insert(body);
                else if (ne->contains(body)) ne->insert(body);
                else if (sw->contains(body)) sw->insert(body);
                else se->insert(body);
            }
            bodies.clear();
        }
    }

    bool contains(const Body* p) const {
        return p->x >= x && p->x < x + width && p->y >= y && p->y < y + height;
    }

    void subdivide() {
        float halfWidth = width / 2.0f;
        float halfHeight = height / 2.0f;

        nw = new Quadtree(x, y, halfWidth, halfHeight);
        ne = new Quadtree(x + halfWidth, y, halfWidth, halfHeight);
        sw = new Quadtree(x, y + halfHeight, halfWidth, halfHeight);
        se = new Quadtree(x + halfWidth, y + halfHeight, halfWidth, halfHeight);

        divided = true;
    }

    void updateCenterOfMass() {
        if (!divided) {
            totalMass = 0.0f;
            centerOfMassX = 0.0f;
            centerOfMassY = 0.0f;
            for (const auto& body : bodies) {
                centerOfMassX += body->x * body->mass;
                centerOfMassY += body->y * body->mass;
                totalMass += body->mass;
            }
            if (totalMass > 0.0f) {
                centerOfMassX /= totalMass;
                centerOfMassY /= totalMass;
            } else {
                centerOfMassX = x + width / 2.0f;
                centerOfMassY = y + height / 2.0f;
            }
        } else {
            nw->updateCenterOfMass();
            ne->updateCenterOfMass();
            sw->updateCenterOfMass();
            se->updateCenterOfMass();
            
            totalMass = nw->totalMass + ne->totalMass + sw->totalMass + se->totalMass;
            if (totalMass > 0.0f) {
                centerOfMassX = (nw->centerOfMassX * nw->totalMass + ne->centerOfMassX * ne->totalMass +
                                 sw->centerOfMassX * sw->totalMass + se->centerOfMassX * se->totalMass) / totalMass;
                centerOfMassY = (nw->centerOfMassY * nw->totalMass + ne->centerOfMassY * ne->totalMass +
                                 sw->centerOfMassY * sw->totalMass + se->centerOfMassY * se->totalMass) / totalMass;
            } else {
                centerOfMassX = x + width / 2.0f;
                centerOfMassY = y + height / 2.0f;
            }
        }
    }
};

struct CollisionPoint {
    float x;
    float y;
    double mass;
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
};

bool isSpacePressed();

void restartExecutable(GLFWwindow* window);

void hsvToRgb(float h, float s, float v, float &r, float &g, float &b);

// void calculateForce(const Body& p1, const Body& p2, double& fx, double& fy);

// void handleCollisions(Body* particle, std::vector<Body*>& massiveParticles, std::vector<Body*>& mobileParticles);

void updateBodies(std::vector<Body>& particles, std::vector<Body*>& massiveParticles, std::vector<Body*>& mobileParticles, double dt);

void drawCollisionDots();

void computeForce(Quadtree* qt, Body* p, float& fx, float& fy);

void calculateForce(Body* body, const Quadtree& quadtree, float dt);

void drawOrbit(const Body& particle, float trailAlpha);

void computePosVel(
    float& x, float& y, float& vx, float& vy, 
    float minVel, float maxVel, float minRadius, float maxRadius,
    std::mt19937& gen);

int run(GLFWwindow* window);

#endif // SIMULATION_H
