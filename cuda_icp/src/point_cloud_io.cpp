#include "point_cloud_io.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <cmath>

Eigen::MatrixXf readPointCloud(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << "\n";
        return Eigen::MatrixXf(0, 3);
    }

    std::vector<Eigen::Vector3f> points;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z;

        if (iss >> x >> y >> z) {
            points.push_back(Eigen::Vector3f(x, y, z));
        }
    }

    file.close();

    // Convert to matrix
    Eigen::MatrixXf result(points.size(), 3);
    for (size_t i = 0; i < points.size(); i++) {
        result.row(i) = points[i];
    }

    return result;
}

void writePointCloud(const std::string& filename, const Eigen::MatrixXf& points) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create file " << filename << "\n";
        return;
    }

    for (int i = 0; i < points.rows(); i++) {
        file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2) << "\n";
    }

    file.close();
    std::cout << "Saved " << points.rows() << " points to " << filename << "\n";
}

Eigen::MatrixXf generateCube(int n_points, float size) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-size/2.0f, size/2.0f);

    Eigen::MatrixXf points(n_points, 3);

    for (int i = 0; i < n_points; i++) {
        // Generate points on the surface of a cube
        int face = i % 6;
        float u = dist(gen);
        float v = dist(gen);

        switch (face) {
            case 0: // +X face
                points(i, 0) = size/2.0f;
                points(i, 1) = u;
                points(i, 2) = v;
                break;
            case 1: // -X face
                points(i, 0) = -size/2.0f;
                points(i, 1) = u;
                points(i, 2) = v;
                break;
            case 2: // +Y face
                points(i, 0) = u;
                points(i, 1) = size/2.0f;
                points(i, 2) = v;
                break;
            case 3: // -Y face
                points(i, 0) = u;
                points(i, 1) = -size/2.0f;
                points(i, 2) = v;
                break;
            case 4: // +Z face
                points(i, 0) = u;
                points(i, 1) = v;
                points(i, 2) = size/2.0f;
                break;
            case 5: // -Z face
                points(i, 0) = u;
                points(i, 1) = v;
                points(i, 2) = -size/2.0f;
                break;
        }
    }

    return points;
}

Eigen::MatrixXf generateSphere(int n_points, float radius) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    Eigen::MatrixXf points(n_points, 3);

    for (int i = 0; i < n_points; i++) {
        // Generate points on sphere surface using spherical coordinates
        float theta = 2.0f * M_PI * dist(gen);  // azimuthal angle
        float phi = std::acos(2.0f * dist(gen) - 1.0f);  // polar angle

        points(i, 0) = radius * std::sin(phi) * std::cos(theta);
        points(i, 1) = radius * std::sin(phi) * std::sin(theta);
        points(i, 2) = radius * std::cos(phi);
    }

    return points;
}

Eigen::MatrixXf transformPointCloud(const Eigen::MatrixXf& points,
                                    const Eigen::Vector3f& translation,
                                    const Eigen::Vector3f& rotation_deg,
                                    float noise_std) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> noise(0.0f, noise_std);

    // Convert rotation from degrees to radians
    Eigen::Vector3f rotation_rad = rotation_deg * M_PI / 180.0f;

    // Create rotation matrices
    Eigen::Matrix3f Rx, Ry, Rz;

    // Rotation around X axis (roll)
    Rx << 1, 0, 0,
          0, std::cos(rotation_rad(0)), -std::sin(rotation_rad(0)),
          0, std::sin(rotation_rad(0)), std::cos(rotation_rad(0));

    // Rotation around Y axis (pitch)
    Ry << std::cos(rotation_rad(1)), 0, std::sin(rotation_rad(1)),
          0, 1, 0,
          -std::sin(rotation_rad(1)), 0, std::cos(rotation_rad(1));

    // Rotation around Z axis (yaw)
    Rz << std::cos(rotation_rad(2)), -std::sin(rotation_rad(2)), 0,
          std::sin(rotation_rad(2)), std::cos(rotation_rad(2)), 0,
          0, 0, 1;

    // Combined rotation (ZYX order)
    Eigen::Matrix3f R = Rz * Ry * Rx;

    // Apply transformation
    Eigen::MatrixXf result = points * R.transpose();

    // Add translation
    for (int i = 0; i < result.rows(); i++) {
        result(i, 0) += translation(0);
        result(i, 1) += translation(1);
        result(i, 2) += translation(2);

        // Add noise if specified
        if (noise_std > 0.0f) {
            result(i, 0) += noise(gen);
            result(i, 1) += noise(gen);
            result(i, 2) += noise(gen);
        }
    }

    return result;
}
