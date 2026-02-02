final: prev:
{
  convex-plane-decomposition = final.callPackage ././src/elevation_mapping_cupy/plane_segmentation/convex_plane_decomposition/package.nix {};
  convex-plane-decomposition-msgs = final.callPackage ././src/elevation_mapping_cupy/plane_segmentation/convex_plane_decomposition_msgs/package.nix {};
  convex-plane-decomposition-ros = final.callPackage ././src/elevation_mapping_cupy/plane_segmentation/convex_plane_decomposition_ros/package.nix {};
  elevation-map-msgs = final.callPackage ././install/elevation_map_msgs/share/elevation_map_msgs/package.nix {};
  elevation-mapping-cupy = final.callPackage ././src/elevation_mapping_cupy/elevation_mapping_cupy/package.nix {};
  grid-map = final.callPackage ././src/grid_map/grid_map/package.nix {};
  grid-map-cmake-helpers = final.callPackage ././install/grid_map_cmake_helpers/share/grid_map_cmake_helpers/package.nix {};
  grid-map-core = final.callPackage ././install/grid_map_core/share/grid_map_core/package.nix {};
  grid-map-costmap-2d = final.callPackage ././src/grid_map/grid_map_costmap_2d/package.nix {};
  grid-map-cv = final.callPackage ././src/grid_map/grid_map_cv/package.nix {};
  grid-map-demos = final.callPackage ././src/grid_map/grid_map_demos/package.nix {};
  grid-map-filters = final.callPackage ././src/grid_map/grid_map_filters/package.nix {};
  grid-map-filters-rsl = final.callPackage ././src/elevation_mapping_cupy/plane_segmentation/grid_map_filters_rsl/package.nix {};
  grid-map-loader = final.callPackage ././src/grid_map/grid_map_loader/package.nix {};
  grid-map-msgs = final.callPackage ././install/grid_map_msgs/share/grid_map_msgs/package.nix {};
  grid-map-octomap = final.callPackage ././src/grid_map/grid_map_octomap/package.nix {};
  grid-map-pcl = final.callPackage ././src/grid_map/grid_map_pcl/package.nix {};
  grid-map-ros = final.callPackage ././src/grid_map/grid_map_ros/package.nix {};
  grid-map-rviz-plugin = final.callPackage ././src/grid_map/grid_map_rviz_plugin/package.nix {};
  grid-map-sdf = final.callPackage ././src/grid_map/grid_map_sdf/package.nix {};
  grid-map-visualization = final.callPackage ././src/grid_map/grid_map_visualization/package.nix {};
  shredder-description = final.callPackage ././src/shredder_description/package.nix {};
}
