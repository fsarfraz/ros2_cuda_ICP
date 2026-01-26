#include "icp_cpu.hpp"
#include "point_cloud_io.hpp"
#include "visualization.hpp"
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>

void printUsage(const char* program_name) {
    std::cout << "CUDA ICP Point-to-Point Alignment\n\n";
    std::cout << "Usage:\n";
    std::cout << "  " << program_name << " <source.txt> <target.txt> [options]\n";
    std::cout << "  " << program_name << " --demo [demo_type]\n\n";
    std::cout << "Options:\n";
    std::cout << "  --max-iter <N>     Maximum iterations (default: 50)\n";
    std::cout << "  --threshold <T>    Convergence threshold (default: 1e-6)\n";
    std::cout << "  --no-viz           Skip visualization\n";
    std::cout << "  --save <file>      Save aligned cloud to file\n";
    std::cout << "  --quiet            Suppress verbose output\n\n";
    std::cout << "Demo modes:\n";
    std::cout << "  translation        Test with pure translation\n";
    std::cout << "  rotation           Test with pure rotation\n";
    std::cout << "  combined           Test with rotation + translation\n";
    std::cout << "  noisy              Test with noise\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << program_name << " source.txt target.txt\n";
    std::cout << "  " << program_name << " --demo translation\n";
    std::cout << "  " << program_name << " source.txt target.txt --max-iter 100 --save aligned.txt\n";
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "  CUDA ICP Point-to-Point Registration  \n";
    std::cout << "========================================\n\n";

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse arguments
    bool demo_mode = false;
    std::string demo_type = "translation";
    std::string source_file, target_file, output_file;
    bool visualize = true;
    bool save_output = false;

    ICPConfig config;
    config.verbose = true;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--demo") {
            demo_mode = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                demo_type = argv[++i];
            }
        } else if (arg == "--max-iter") {
            if (i + 1 < argc) {
                config.max_iterations = std::atoi(argv[++i]);
            }
        } else if (arg == "--threshold") {
            if (i + 1 < argc) {
                config.convergence_threshold = std::atof(argv[++i]);
            }
        } else if (arg == "--no-viz") {
            visualize = false;
        } else if (arg == "--save") {
            if (i + 1 < argc) {
                save_output = true;
                output_file = argv[++i];
            }
        } else if (arg == "--quiet") {
            config.verbose = false;
        } else if (source_file.empty()) {
            source_file = arg;
        } else if (target_file.empty()) {
            target_file = arg;
        }
    }

    Eigen::MatrixXf source, target;

    // Load or generate point clouds
    if (demo_mode) {
        std::cout << "Demo mode: " << demo_type << "\n\n";

        // Generate synthetic data
        source = generateSphere(5000, 1.0f);
        std::cout << "Generated source sphere with " << source.rows() << " points\n";

        if (demo_type == "translation") {
            target = transformPointCloud(source, Eigen::Vector3f(0.3f, 0.2f, -0.1f),
                                         Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0f);
            std::cout << "Applied translation: (0.3, 0.2, -0.1)\n\n";
        } else if (demo_type == "rotation") {
            target = transformPointCloud(source, Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                         Eigen::Vector3f(0.0f, 0.0f, 30.0f), 0.0f);
            std::cout << "Applied rotation: 30 degrees around Z axis\n\n";
        } else if (demo_type == "combined") {
            target = transformPointCloud(source, Eigen::Vector3f(0.2f, 0.15f, -0.1f),
                                         Eigen::Vector3f(10.0f, 15.0f, 20.0f), 0.0f);
            std::cout << "Applied rotation: (10, 15, 20) degrees\n";
            std::cout << "Applied translation: (0.2, 0.15, -0.1)\n\n";
        } else if (demo_type == "noisy") {
            target = transformPointCloud(source, Eigen::Vector3f(0.2f, 0.1f, -0.05f),
                                         Eigen::Vector3f(0.0f, 0.0f, 15.0f), 0.01f);
            std::cout << "Applied rotation: 15 degrees around Z axis\n";
            std::cout << "Applied translation: (0.2, 0.1, -0.05)\n";
            std::cout << "Added Gaussian noise with std=0.01\n\n";
        } else {
            std::cerr << "Unknown demo type: " << demo_type << "\n";
            return 1;
        }
    } else {
        // Load from files
        if (source_file.empty() || target_file.empty()) {
            std::cerr << "Error: Source and target files required\n\n";
            printUsage(argv[0]);
            return 1;
        }

        std::cout << "Loading point clouds...\n";
        source = readPointCloud(source_file);
        target = readPointCloud(target_file);

        if (source.rows() == 0 || target.rows() == 0) {
            std::cerr << "Error: Failed to load point clouds\n";
            return 1;
        }

        std::cout << "Loaded " << source.rows() << " source points\n";
        std::cout << "Loaded " << target.rows() << " target points\n\n";
    }

    // Run ICP
    std::cout << "Running ICP algorithm...\n";
    std::cout << "========================================\n\n";

    auto result = computeICP(source, target, config);

    // Print results
    std::cout << "\n========================================\n";
    std::cout << "  ICP Results\n";
    std::cout << "========================================\n";
    std::cout << "Converged: " << (result.converged ? "Yes" : "No") << "\n";
    std::cout << "Iterations: " << result.iterations << "\n";
    std::cout << "Final RMSE: " << result.final_error << "\n\n";

    std::cout << "Transformation Matrix:\n";
    std::cout << result.transformation << "\n\n";

    // Apply transformation to source
    Eigen::MatrixXf aligned = applyTransformation(source, result.transformation);

    // Save if requested
    if (save_output) {
        writePointCloud(output_file, aligned);
    }

    // Visualize if requested
    if (visualize) {
        std::cout << "Launching visualization...\n";
        visualizeAlignment(source, target, aligned, result.transformation);
    }

    std::cout << "Done!\n";
    return 0;
}
