
#include "elevation_map.h"

ElevationMap::ElevationMap(const ros::NodeHandle& nh, const DMap::dmap::Ptr& dmap_ptr) {
  nh_ = nh;
  dmap_ptr_ = dmap_ptr;

  elevation_map_ptr_.reset(new grid_map::GridMap ({"elevation"}));
  elevation_map_ptr_->setFrameId("world");
  elevation_map_ptr_->setBasicLayers({"elevation"});

  auto bbx = dmap_ptr_->GetBBX();
  grid_map::Length map_size(bbx.vertex_max[0] - bbx.vertex_min[0],
                            bbx.vertex_max[1] - bbx.vertex_min[1]);

  elevation_map_ptr_->setGeometry(map_size,
                                  dmap_ptr->getResolution(),
                                  grid_map::Position(0.0, 0.0));
  // TODO: set robot size and height
  robot_height_ = 1.0;

  elevation_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  occ_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map", 2);
}

void ElevationMap::UpdateElevationMap() {
  auto start_time = std::chrono::steady_clock::now();
  PointVector occupied_points;
  dmap_ptr_->occupied_pointcloud_visualize(occupied_points);
  for (auto& point : occupied_points) {
    // Check if the point is above the robot, then skip
    if(point.z > robot_height_){
      continue;
    }
    // Get the elevation of the point
    auto z_value = elevation_map_ptr_->atPosition("elevation", grid_map::Position(point.x, point.y));
//    std::cout << "z value: " << z_value << std::endl;
    if(std::isnan(z_value) || z_value < point.z){
//      std::cout << "update" << std::endl;
      elevation_map_ptr_->atPosition("elevation", grid_map::Position(point.x, point.y)) = point.z;
    }
  }

  auto duration = std::chrono::steady_clock::now() - start_time;
  std::cout << "Update elevation: " << duration.count()  * 1e-6 << " ms " << std::endl;

  if(vis_en_){
    PublishElevationMap();
  }
}

void ElevationMap::PublishElevationMap() {
  grid_map::GridMapRosConverter::toMessage(*elevation_map_ptr_, elevation_map_msg_);
  elevation_map_pub_.publish(elevation_map_msg_);
}
