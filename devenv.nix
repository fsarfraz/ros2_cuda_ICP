{
  pkgs,
  lib,
  config,
  nixpkgs,
  nix-ros-overlay,
  nixgl,
  ...
}:

let
  # nixGL setup for GPU support outside NixOS
  isIntelX86Platform = pkgs.stdenv.system == "x86_64-linux";
  nixGL = import nixgl {
    inherit pkgs;
    enable32bits = isIntelX86Platform;
    enableIntelX86Extensions = isIntelX86Platform;
  };

  eigen_stable = pkgs.stdenv.mkDerivation rec {
    pname = "eigen";
    version = "3.4.0";

    src = pkgs.fetchFromGitLab {
      owner = "libeigen";
      repo = "eigen";
      rev = version;
      sha256 = "sha256-1/4xMetKMDOgZgzz3WMxfHUEpmdAm52RqZvz6i0mLEw=";
    };

    nativeBuildInputs = [ pkgs.cmake ];
    dontBuild = true;
  };

  gtsam = pkgs.stdenv.mkDerivation rec {
    pname = "gtsam";
    version = "4.2.0";

    src = pkgs.fetchFromGitHub {
      owner = "borglab";
      repo = "gtsam";
      rev = version;
      sha256 = "sha256-HjpGrHclpm2XsicZty/rX/RM/762wzmj4AAoEfni8es=";
    };

    nativeBuildInputs = with pkgs; [ cmake ];
    buildInputs = with pkgs; [
      boost
      eigen_stable
      tbb_2021_11
    ];
    cmakeFlags = [
      "-DGTSAM_BUILD_TESTS=OFF"
      "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
      "-DGTSAM_BUILD_PYTHON=OFF"
      "-DGTSAM_USE_SYSTEM_EIGEN=ON"
      "-DEIGEN3_INCLUDE_DIR=${eigen_stable}/include/eigen3"
      "-DGTSAM_WITH_TBB=ON"
      "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF"
      "-DGTSAM_USE_BOOST_FEATURES=OFF"
    ];
  };

in
{
  name = "ros-workspace";

  cachix.pull = [ "ros" "cuda-maintainers" ];

  overlays = [
    nix-ros-overlay.overlays.default
  ];

  env.CARGO_HOME = "${config.env.DEVENV_ROOT}/.cargo";

  packages =
    with pkgs;
    [
      # Essential dev tools
      python3
      python3Packages.argcomplete
      python3Packages.rosdep
      python3Packages.pytest
      python3Packages.pyyaml
      python3Packages.pyzmq
      python3Packages.transforms3d
      python3Packages.ruamel-yaml
      python3Packages.scipy
      python3Packages.shapely
      python3Packages.simple-parsing
      python3Packages.pip
      python3Packages.virtualenv
      # cupy and torch - use pip in venv for prebuilt CUDA wheels (faster)


      # Build tools
      git
      colcon
      cmake
      ninja
      ccache
      pkg-config
      cargo
      rustc
      clang
      llvmPackages.libclang
      llvmPackages.openmp
      lcov
      mcap-cli

      # CUDA toolkit (modular packages)
      cudatoolkit
      cudaPackages.cuda_nvcc
      cudaPackages.cuda_cudart      # CUDA runtime (includes libcudart_static, libcudadevrt)
      cudaPackages.cuda_cccl        # CUDA C++ Core Libraries
      cudaPackages.libcublas        # CUDA BLAS library
      cudaPackages.libcufft         # CUDA FFT library

      # nixGL for NVIDIA driver compatibility (uses system drivers)
      # nixGL.auto.nixGLNvidia
      # nixGL.auto.nixGLNvidiaBumblebee
      nixGL.auto.nixGLDefault
      # nixGL.nixGLIntel

      # Graphics and visualization
      graphviz
      cairo
      graphicsmagick
      qt5.qtbase
      xorg.libXinerama
      xorg.libX11
      xorg.libXrandr
      xorg.libXxf86vm
      xorg.libXcursor
      xorg.libXi
      xorg.libXext
      libGL
      libGLU
      freeglut
      glfw
      glew
      wxGTK32
      mesa
      # Math and scientific computing
      boost
      # eigen
      eigen_stable
      tbb_2021_11
      (ceres-solver.override { eigen = eigen_stable; })
      suitesparse
      lapack
      blas
      nanoflann
      xsimd
      xtensor
      gbenchmark

      # Computer vision and point clouds
      opencv
      pcl

      # Geometry and mapping
      cgal
      gmp
      mpfr
      geographiclib
      orocos-kdl
      octomap
      assimp

      # Networking and serialization
      curl
      libtins
      jsoncpp
      nlohmann_json
      yaml-cpp

      # Compression and data
      lz4
      flann
      spdlog
      zlib

      # Media
      ffmpeg
      libjpeg
      libpng
      tinyxml-2

      # Hardware/USB
      libusb1
      udev
      cli11
      # Custom builds
      gtsam
    ]
    ++ (with pkgs.rosPackages.jazzy; [
      (buildEnv {
        name = "ros-env";
        paths = [
          # Core ROS
          ros-core
          ros-environment
          ros-gz

          # Build system
          ament-cmake
          ament-cmake-core
          ament-cmake-ros
          ament-cmake-cpplint
          ament-cmake-flake8
          ament-cmake-gmock
          ament-cmake-gtest
          ament-cmake-lint-cmake
          ament-cmake-pep257
          ament-cmake-pytest
          ament-cmake-python
          ament-cmake-uncrustify
          ament-cmake-xmllint
          ament-copyright
          ament-flake8
          ament-lint-auto
          ament-lint-cmake
          ament-lint-common
          ament-pep257
          ament-index-cpp
          ament-index-python
          python-cmake-module
          # eigen3-cmake-module

          # Core ROS communication
          rclcpp
          rclcpp-action
          rclcpp-components
          rclcpp-lifecycle
          rclpy
          rcl-interfaces
          rcutils
          rmw-implementation

          # Message types
          action-msgs
          builtin-interfaces
          std-msgs
          std-srvs
          geometry-msgs
          sensor-msgs
          nav-msgs
          map-msgs
          test-msgs
          visualization-msgs
          diagnostic-msgs
          lifecycle-msgs
          bond
          bondcpp

          # TF and transforms
          tf2
          tf2-ros
          tf2-eigen
          tf2-geometry-msgs
          tf2-msgs
          tf2-sensor-msgs
          tf2-kdl
          tf-transformations

          # Perception
          cv-bridge
          image-transport
          pcl-conversions
          pcl-ros
          laser-geometry

          # Navigation and control
          angles
          behaviortree-cpp
          ompl
          diff-drive-controller
          joint-state-broadcaster
          joint-state-publisher
          joint-state-publisher-gui

          # Visualization
          rviz2
          rviz-common
          rviz-default-plugins
          rviz-ogre-vendor
          rviz-rendering
          interactive-markers

          # Launch and lifecycle
          launch
          launch-ros
          launch-testing
          launch-testing-ament-cmake
          launch-testing-ros

          # Utilities
          backward-ros
          diagnostic-updater
          geographic-msgs
          message-filters
          pluginlib
          resource-retriever
          robot-state-publisher
          urdf
          xacro
          yaml-cpp-vendor

          # Data recording
          rosbag2
          rosbag2-cpp
          rosbag2-storage
          rosbag2-storage-mcap
          rosbag2-transport

          # Code generation
          rosidl-default-generators
          rosidl-default-runtime

          # Middleware
          rmw-cyclonedds-cpp

          # Development tools
          osrf-pycommon

          # Additional packages from ros2nix
          kdl-parser
          orocos-kdl-vendor
          nav2-common
          nav2-msgs
          nav2-costmap-2d

          # Camera/Image packages
          camera-calibration-parsers
          camera-info-manager

          # Filters
          filters

          # Navigation
          slam-toolbox
          teleop-twist-keyboard

          # Octomap
          octomap-msgs
          octomap-rviz-plugins

          # Rosbag extensions
          rosbag2-py
          rosbag2-storage-default-plugins

        ];
      })
    ]);

  scripts.zenoh = {
    exec = ''
      echo "Launching Zenoh bridge"
      source install/setup.bash && zenoh_bridge_ros2dds 
      #-c zenoh-bridge-config.json
    '';
  };

  scripts.rviz = {
    exec = ''
      echo "Launching RViz"
      source install/setup.bash && ros2 run rviz2 rviz2
    '';
  };

  scripts.cbuild = {
    exec = ''
      echo "Building workspace with colcon"
      colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install
    '';
  };

  scripts.lidar = {
    exec = ''
      echo "Launching lidar driver"
      source install/setup.bash && ros2 launch ouster_ros driver.launch.py
    '';
  };

  scripts.install-cuda-python = {
    exec = ''
      echo "Installing CUDA Python packages in venv..."
      # Use workspace dir for temp to avoid /tmp space issues
      mkdir -p .pip-tmp
      TMPDIR="$(pwd)/.pip-tmp" pip install --no-cache-dir cupy-cuda12x torch --extra-index-url https://download.pytorch.org/whl/cu121
      rm -rf .pip-tmp
      echo "Done! cupy and torch with CUDA installed."
    '';
  };

  # containers.zenoh = {
  #   name = "zenoh";
  #   startupCommand = config.scripts.zenoh.exec;
  # };

 enterShell = ''
  # Create CARGO_HOME directory if it doesn't exist
  mkdir -p ${config.env.DEVENV_ROOT}/.cargo

  # Python venv for pip packages (cupy, torch with CUDA)
  VENV_DIR="${config.env.DEVENV_ROOT}/.venv"
  if [ ! -d "$VENV_DIR" ]; then
    echo "Creating Python venv at $VENV_DIR..."
    python -m venv "$VENV_DIR" --system-site-packages
  fi
  source "$VENV_DIR/bin/activate"
  export PYTHONPATH="$VENV_DIR/lib/python3.12/site-packages:$PYTHONPATH"

  # X11 Display for GUI applications
  export DISPLAY=''${DISPLAY:-:0}

  # OpenGL/GLX configuration for NVIDIA
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __VK_LAYER_NV_optimus=NVIDIA_only
  export LIBGL_ALWAYS_INDIRECT=0

  # Eigen configuration
  export EIGEN3_INCLUDE_DIR=${eigen_stable}/include/eigen3
  export CMAKE_PREFIX_PATH="${pkgs.tbb_2021_11}:$CMAKE_PREFIX_PATH"
  
  # CUDA configuration - use cudaPackages for consistency
  export CUDA_PATH="${pkgs.cudaPackages.cudatoolkit}"
  export CUDA_HOME="${pkgs.cudaPackages.cudatoolkit}"
  export CMAKE_CUDA_COMPILER="${pkgs.cudaPackages.cuda_nvcc}/bin/nvcc"
  export PATH="${pkgs.cudaPackages.cuda_nvcc}/bin:$PATH"

  # nixGL setup - wraps OpenGL/Vulkan apps to use system NVIDIA drivers
  export NIXGL_PREFIX="${nixGL.auto.nixGLDefault}/bin/nixGL"

  # CUDA library paths (nixGL handles OpenGL/Vulkan driver libs automatically)
  export LD_LIBRARY_PATH="${pkgs.cudaPackages.cudatoolkit}/lib:$LD_LIBRARY_PATH"
  export EXTRA_LDFLAGS="-L${pkgs.cudaPackages.cudatoolkit}/lib"
  export EXTRA_CCFLAGS="-I${pkgs.cudaPackages.cudatoolkit}/include"
  
  # Compiler flags for compatibility
  export NIX_CFLAGS_COMPILE="$NIX_CFLAGS_COMPILE -Wno-error=unused-result -Wno-error=array-bounds"
  export NIX_CFLAGS_COMPILE="$NIX_CFLAGS_COMPILE -Wno-error=implicit-function-declaration -Wno-error=int-conversion -Wno-error=template-id-cdtor"
  
  # Clang configuration
  export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
  
  # ROS configuration
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=1
  
  # Shell completions
  eval "$(register-python-argcomplete ros2)"
  eval "$(register-python-argcomplete colcon)"
  
  # Cleanup on exit
  trap 'ros2 daemon stop' EXIT
  '';

  # Temporarily disabled due to nixf-diagnose dependency issue
  # git-hooks.hooks = {
  #   shellcheck.enable = true;
  #   flake-checker.enable = true;
  #   nixfmt-rfc-style.enable = true;
  #   actionlint.enable = true;
  # };
}
