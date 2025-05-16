#include <iostream>

#include "particle_filter.hpp"

constexpr float TWO_PI = 2.0f * M_PI;

Map::Map() {
    walls_ = {
        {{0.0f, 0.0f}, {10.0f, 0.0f}},   // Bottom wall
        {{10.0f, 0.0f}, {10.0f, 10.0f}}, // Right wall
        {{10.0f, 10.0f}, {0.0f, 10.0f}}, // Top wall
        {{0.0f, 10.0f}, {0.0f, 0.0f}}    // Left wall
    };
    landmarks_ = {
        {0, 0},
        {0, 10},
        {10, 10},
        {10, 0}
    };
}

const std::vector<Wall>& Map::getWalls() const {
    return walls_;
}

const std::vector<Eigen::Vector2f>& Map::getLandmarks() const{
    return landmarks_;
}

Robot::Robot(float sigma_pos, float sigma_rot, float sigma_sense, float x, float y, float theta) : x_(x), y_(y), theta_(theta), sigma_pos_(0.1f), sigma_rot_(0.05f), sigma_sense_(0.05f), dist_pos_(0.0f, sigma_pos_), dist_rot_(0.0f, sigma_rot_), dist_sense_(0.0f, sigma_sense_) {}

void Robot::moveForward(float distance)
{
    float noisy_distance {distance + dist_pos_(gen_)};
    x_ += noisy_distance * cos(theta_);
    y_ += noisy_distance * sin(theta_);
}

void Robot::rotate(float rotation)
{
    float noisy_rotation {rotation + dist_rot_(gen_)};
    theta_ = fmod(theta_ + noisy_rotation, TWO_PI);
    if(theta_ < 0) theta_ += TWO_PI; // Keep theta in range (0 - 2π)
}

std::vector<float> Robot::senseAllLandmarks(const std::vector<Eigen::Vector2f>& landmarks)
{
    std::vector<float> measurements;
    for (const auto& lm : landmarks)
    {
        float dx = lm[0] - x_;
        float dy = lm[1] - y_;
        float dist = std::sqrt(dx * dx + dy * dy);
        measurements.push_back(dist + dist_sense_(gen_));
    }
    return measurements;
}

void Robot::printState() const
{
    std::cout << "Robot state: x = " << x_
              << ", y = " << y_
              << ", theta = " << theta_ << std::endl;
}

ParticleFilter::ParticleFilter(float sigma_pos, float sigma_rot, float sigma_sense) : sigma_pos_(sigma_pos), sigma_rot_(sigma_rot), sigma_sense_(sigma_sense), dist_pos_(0.0f, sigma_pos_), dist_rot_(0.0f, sigma_rot), dist_sense_(0.0f, sigma_sense_) {}

void ParticleFilter::initializeParticles(float robot_x, float robot_y, float robot_theta, int num_particles)
{
    std::normal_distribution<float> normal_x(robot_x, sigma_pos_);
    std::normal_distribution<float> normal_y(robot_y, sigma_pos_);
    std::normal_distribution<float> normal_theta(robot_theta, sigma_rot_);

    for(int i{0}; i < num_particles; ++i)
    {
        Particle p{normal_x(gen_), normal_y(gen_), normal_theta(gen_), 1.0f/num_particles};
        particles_.push_back(p);
    }
}

void ParticleFilter::moveParticle(Particle& p, float distance, float rotation)
{
    float noisy_distance {distance + dist_pos_(gen_)};
    p.x += noisy_distance * cos(p.theta);
    p.y += noisy_distance * sin(p.theta);

    float noisy_rotation {rotation + dist_rot_(gen_)};
    p.theta = fmod(p.theta + noisy_rotation, TWO_PI);
    if(p.theta < 0) p.theta += TWO_PI;
}

void ParticleFilter::updateMotion(float distance, float rotation)
{
    for (auto& p : particles_)
    {
        moveParticle(p, distance, rotation);
    }
}

void ParticleFilter::calculateWeights(const std::vector<float>& measurements, const std::vector<Eigen::Vector2f>& landmarks)
{
    float total_weight{0.0f};
    const float gauss_norm = static_cast<float>(1.0 / std::sqrt(2.0 * M_PI * sigma_sense_ * sigma_sense_));
    const float denom{2.0f * sigma_sense_ * sigma_sense_};

    for (auto& p : particles_) {
        float weight{1.0f};

        for (std::size_t i = 0; i < landmarks.size(); ++i) {
            const auto& lm = landmarks[i];
            float expected = std::sqrt((lm[0] - p.x) * (lm[0] - p.x) + (lm[1] - p.y) * (lm[1] - p.y));
            float diff = measurements[i] - expected;

            // Multiply the probability of each measurement (assuming independence)
            weight *= gauss_norm * std::exp(-(diff * diff) / denom);
        }

        p.weight = weight;
        total_weight += p.weight;
    }

    if (total_weight > 0.0f) {
        for (auto& p : particles_)
            p.weight /= total_weight;
    }
}

void ParticleFilter::resampleParticles()
{
    std::vector<Particle> new_particles;
    std::vector<float> weights;
    for (const auto& p : particles_)
        weights.push_back(p.weight);

    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    
    for (size_t i{0}; i < particles_.size(); ++i)
    {
        int index {distribution(gen_)};
        new_particles.push_back(particles_[index]);
    }
    particles_ = std::move(new_particles);
}

Eigen::Vector3f ParticleFilter::estimateState() const
{
    Eigen::Vector3f estimated_pose;
    float sum_x{0}, sum_y{0}, sum_sin{0}, sum_cos{0};
    for (const auto& p : particles_) {
        sum_x += p.x;
        sum_y += p.y;
        sum_sin += sin(p.theta);
        sum_cos += cos(p.theta);
    }
    estimated_pose[0] = sum_x/particles_.size();
    estimated_pose[1] = sum_y/particles_.size();
    estimated_pose[2] = atan2(sum_sin/particles_.size(), sum_cos/particles_.size());

    return estimated_pose;
}

Eigen::Vector3f ParticleFilter::updateAndEstimate(float distance, float rotation, const std::vector<float>& measurements, const std::vector<Eigen::Vector2f>& landmarks)
{
    // 1. Move particles
    updateMotion(distance, rotation);

    // 2. Update weights based on measurement
    calculateWeights(measurements, landmarks);

    // 3. Resample particles (if needed)
    float total_weight_squared {0};
    for (const auto& p : particles_)
        total_weight_squared += p.weight * p.weight;

    float Neff {1.0f / (total_weight_squared + 1e-6f)}; // Avoid division by 0
    if (Neff < particles_.size() / 2) // If Neff is less than half the number of particles (too small)
        resampleParticles();

    // 4. Estimate position of the robot
    Eigen::Vector3f estimated_pose {estimateState()};

    return estimated_pose;
}

int main()
{
    float sigma_pos {0.1f}, sigma_rot {0.05f}, sigma_sense {0.3f};
    float x {1.0f}, y {1.0f}, theta {0.0f};
    int num_particles {500};

    Map map;
    Robot robot(sigma_pos, sigma_rot, sigma_sense, x, y, theta);
    ParticleFilter pf(robot.getSigmaPos(), robot.getSigmaRot(), robot.getSigmaSense());

    pf.initializeParticles(x, y, theta, num_particles);

    // The robot will start in position 1,1
    std::vector<int> side_lengths = {8, 8, 8, 8};
    float forward_distance = 1.0f;
    float turn_angle = -M_PI / 2;  // Right turn

    for (int side {0}; side < 4; ++side)
    {
        for (int step{0}; step < side_lengths[side]; ++step)
        {
            robot.moveForward(forward_distance);
            std::vector<float> measurements = robot.senseAllLandmarks(map.getLandmarks());
            Eigen::Vector3f estimate = pf.updateAndEstimate(forward_distance, 0.0f, measurements, map.getLandmarks());
            robot.printState();
            std::cout << "Estimated state: x = " << estimate[0]
                    << ", y = " << estimate[1]
                    << ", theta = " << estimate[2] << std::endl;
        }
        robot.rotate(turn_angle);
        std::vector<float> measurements = robot.senseAllLandmarks(map.getLandmarks());
        Eigen::Vector3f estimate = pf.updateAndEstimate(0.0f, turn_angle, measurements, map.getLandmarks());
        robot.printState();
        std::cout << "Estimated state: x = " << estimate[0]
                << ", y = " << estimate[1]
                << ", theta = " << estimate[2] << std::endl;
    }
}