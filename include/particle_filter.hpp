#pragma once
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <cmath>

struct Wall {
    Eigen::Vector2f start;
    Eigen::Vector2f end;
};

struct Particle{
    float x, y, theta, weight;
};

class Map {
public:
    Map();
    const std::vector<Wall>& getWalls() const;
    const std::vector<Eigen::Vector2f>& getLandmarks() const;

private:
    std::vector<Wall> walls_;
    std::vector<Eigen::Vector2f> landmarks_;
};

class Robot {
public:
    Robot(float sigma_pos, float sigma_rot, float sigma_sense, float x, float y, float theta);

    void moveForward(float distance);
    void rotate(float angle);
    std::vector<float> senseAllLandmarks(const std::vector<Eigen::Vector2f>& landmarks);

    // Getter functions
    float getSigmaPos() const { return sigma_pos_; }
    float getSigmaRot() const { return sigma_rot_; }
    float getSigmaSense() const { return sigma_sense_; }

    // Print state (for testing)
    void printState() const;

private:
    float x_, y_, theta_;
    float sigma_pos_; // standard deviation for position
    float sigma_rot_; // standard deviation for rotation
    float sigma_sense_; // standard deviation for sensing

    float normalizeAngle(float theta);

    std::default_random_engine gen_; // Random number generation
    std::normal_distribution<float> dist_pos_; // Noise for position
    std::normal_distribution<float> dist_rot_; // Noise for rotation
    std::normal_distribution<float> dist_sense_; // Noise for sensing
};

class ParticleFilter{
public:
    ParticleFilter(float sigma_pos, float sigma_rot, float sigma_sense);

    void initializeParticles(float robot_x, float robot_y, float robot_theta, int num_particles);
    Eigen::Vector3f updateAndEstimate(float distance, float rotation, const std::vector<float>& measurements, const std::vector<Eigen::Vector2f>& landmarks);

private:
    std::vector<Particle> particles_;
    float sigma_pos_, sigma_rot_, sigma_sense_;
    std::default_random_engine gen_;
    std::normal_distribution<float> dist_pos_;
    std::normal_distribution<float> dist_rot_;
    std::normal_distribution<float> dist_sense_;

    void moveParticle(Particle& p, float distance, float rotation);
    void updateMotion(float distance, float rotation);
    void calculateWeights(const std::vector<float>& measurements, const std::vector<Eigen::Vector2f>& landmarks);
    void resampleParticles();
    Eigen::Vector3f estimateState() const;
};