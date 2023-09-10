//
// Created by kevinlad on 23-9-8.
//

#ifndef SRC_D_MAP_SRC_GRID_MAP_HELPER_H_
#define SRC_D_MAP_SRC_GRID_MAP_HELPER_H_

#include <grid_map_ros/grid_map_ros.hpp>
#include <D-Map/D-Map.h>

class ElevationMap {
 public:
  using Ptr = std::shared_ptr<ElevationMap>;
  ElevationMap(const ros::NodeHandle& nh, const DMap::dmap::Ptr& dmap_ptr);

  void UpdateElevationMap();

  void PublishElevationMap();

 private:
  ros::NodeHandle nh_;
  DMap::dmap::Ptr dmap_ptr_;
  std::shared_ptr<grid_map::GridMap> elevation_map_ptr_;
  Vector3d robot_size_;
  double robot_height_;

  ros::Publisher elevation_map_pub_, occ_map_pub_;
  grid_map_msgs::GridMap elevation_map_msg_, occ_map_msg_;
  bool vis_en_ = true;
};

#endif //SRC_D_MAP_SRC_GRID_MAP_HELPER_H_
