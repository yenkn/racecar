//
// Created by yenkn on 19-7-1.
//
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <racecar_core/point.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <racecar_core/utils/tf.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MapLocalizationNode {
public:
  MapLocalizationNode();
  ~MapLocalizationNode();

private:
  ros::NodeHandle nh_;
  ros::Subscriber mapSub_, scanSub_, odomSub_;
  ros::Publisher posePub_, scanPub_, cloudPub_;
  tf::TransformListener tfListener_;
  tf::TransformBroadcaster tfBroadcaster_;
  nav_msgs::OccupancyGrid map_;
  nav_msgs::Odometry odom_;
  sensor_msgs::LaserScan scan_;

  tf::Transform laserToBase_;
  tf::Pose estimatePose_;

  PointCloudT::Ptr targetMapPtr_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;

  std::ofstream fout_;

  void mapCallback(const nav_msgs::OccupancyGridConstPtr &mapMsg);
  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr &scanMsg);

};

MapLocalizationNode::MapLocalizationNode(): nh_("~"), targetMapPtr_(new PointCloudT()), fout_("~/catkin_ws/src/data.json") {
  mapSub_ = nh_.subscribe("/map", 1, &MapLocalizationNode::mapCallback, this);
  scanSub_ = nh_.subscribe("/scan", 1, &MapLocalizationNode::scanCallback, this);
  odomSub_ = nh_.subscribe("/odometry/filtered", 1, &MapLocalizationNode::odomCallback, this);

  posePub_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimate_pose", 1);
  scanPub_ = nh_.advertise<nav_msgs::GridCells>("/sim_scan", 1);
  cloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);
  estimatePose_ = utils::createTfFromXYTheta(0, 0, 0);
  fout_ << "{\n";
}


MapLocalizationNode::~MapLocalizationNode() {
  fout_ << "}";
  fout_.close();
}

void MapLocalizationNode::mapCallback(const nav_msgs::OccupancyGridConstPtr &mapMsg) {
  map_ = *mapMsg;

  fout_ << "\"map\": [";
  targetMapPtr_->clear();
  size_t len = map_.data.size();
  for(int i = 0; i < len; i++) {
    if(map_.data[i] == 100) {
      PointT pt(
          map_.info.origin.position.x + (i % map_.info.width) * map_.info.resolution,
          map_.info.origin.position.y + floor(i / map_.info.width) * map_.info.resolution,
          0);
      targetMapPtr_->push_back(pt);
      fout_ << '[' << pt.x << ',' << pt.y << ']';
      if(i < len - 1) fout_ << ',';
    }
  }
  fout_ << "], \n\"scan\": [";

  PointCloudT::Ptr output_cloud(new PointCloudT());
  ndt_.setResolution(1.0);
  ndt_.setInputTarget(targetMapPtr_);
  ndt_.setMaximumIterations(30);
  ndt_.setStepSize(0.1);
  ndt_.setTransformationEpsilon(0.01);
  ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());


  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud(output_cloud);
  while (!viewer.wasStopped ())
  {
  }
}

void MapLocalizationNode::odomCallback(const nav_msgs::OdometryConstPtr &odomMsg) {
  if(odom_.header.seq == 0) {
    odom_ = *odomMsg;
    return;
  }
  odom_ = *odomMsg;

}

void MapLocalizationNode::scanCallback(const sensor_msgs::LaserScanConstPtr &scanMsg) {
  if(scan_.header.seq == 0) {
    tf::StampedTransform baseToLaser;
    if(!utils::getTFTransform(tfListener_, "base_footprint", scanMsg->header.frame_id, baseToLaser)) {
      ROS_ERROR("Unable to get base to laser transform");
      return;
    }
    laserToBase_ = baseToLaser.inverse();
    // tfBroadcaster_.sendTransform(tf::StampedTransform(utils::createTfFromXYTheta(0, 0, 0), ros::Time::now(), "map", "odom"));
    scan_ = *scanMsg;
    return;
  }

  fout_ << "\n\t{\"data\": [";
  scan_ = *scanMsg;
  pcl::PointCloud<PointT>::Ptr localScan(new PointCloudT());
  for(int i = 0; i < scan_.ranges.size(); i++) {
    if (scan_.ranges[i] < scan_.range_min || scan_.ranges[i] >= scan_.range_max) continue;
    auto angle = scan_.angle_min + i * scan_.angle_increment;
    auto point = laserToBase_ * utils::polarToCartesian(scan_.ranges[i], angle);
    localScan->push_back(PointT(point.x(), point.y(), point.z()));
    fout_ << '[' << point.x() << ',' << point.y() << ']';
    if(i < scan_.ranges.size() - 1) fout_ << ',';
  }
  fout_ << "],";

  PointCloudT::Ptr scan_ptr(new PointCloudT());
  pcl::VoxelGrid<PointT> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
  voxel_grid_filter.setInputCloud(localScan);
  voxel_grid_filter.filter(*scan_ptr);

  tf::StampedTransform mapToCar;
  if(!utils::getTFTransform(tfListener_, "map", "base_footprint", mapToCar)) {
    return;
  }
  auto carPose = mapToCar.inverse();

  double roll, pitch, yaw;
  tf::Matrix3x3(carPose.getRotation()).getRPY(roll, pitch, yaw);
  std::cout << "initial: " << carPose.getOrigin().x() << ", " << carPose.getOrigin().y() << ", " << yaw << std::endl;
  Eigen::Translation3f init_translation(carPose.getOrigin().x(), carPose.getOrigin().y(), carPose.getOrigin().z());
  Eigen::AngleAxisf init_rotation_x(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f initGuess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
  fout_ << "\"guess\": [" << carPose.getOrigin().x() << "," << carPose.getOrigin().y() << "," << yaw << "]}";

  ndt_.setInputSource(scan_ptr);
  ndt_.setInputTarget(targetMapPtr_);

  PointCloudT::Ptr outputCloud(new PointCloudT());
  auto alignStart = ros::Time::now();
  ndt_.align(*outputCloud);
  auto alignEnd = ros::Time::now();

  auto finalTF = ndt_.getFinalTransformation();
//  has_converged_ = ndt_.hasConverged();
//  iteration_ = ndt_.getFinalNumIteration();
//  trans_probability_ = ndt_.getTransformationProbability();

  auto fitnessScore = ndt_.getFitnessScore();

  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(finalTF(0, 0)), static_cast<double>(finalTF(0, 1)), static_cast<double>(finalTF(0, 2)),
                 static_cast<double>(finalTF(1, 0)), static_cast<double>(finalTF(1, 1)), static_cast<double>(finalTF(1, 2)),
                 static_cast<double>(finalTF(2, 0)), static_cast<double>(finalTF(2, 1)), static_cast<double>(finalTF(2, 2)));
  tf::Quaternion q;
  mat_b.getRotation(q);

  tf::Transform finalTransform;
  finalTransform.setOrigin(tf::Vector3(finalTF(0, 3), finalTF(1, 3), finalTF(2, 3)));
  finalTransform.setRotation(q);

  std::cout << "final: " << finalTF(0, 3) << ", " << finalTF(1, 3) << ", " << tf::getYaw(q) << std::endl;
  std::cout << "fitness: " << fitnessScore << ", time: " << (alignEnd - alignStart).toSec() << ", converged: " << ndt_.hasConverged() << ", iteration: " << ndt_.getFinalNumIteration() << std::endl;

  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*outputCloud, cloudMsg);
  cloudMsg.header.stamp = ros::Time::now();
  cloudMsg.header.frame_id = "map";
  cloudPub_.publish(cloudMsg);

  geometry_msgs::PoseStamped poseMsg;
  tf::poseTFToMsg(finalTransform, poseMsg.pose);
  poseMsg.header.frame_id = "map";
  poseMsg.header.stamp = ros::Time::now();
  posePub_.publish(poseMsg);

}
/*
size_t MapLocalizationNode::simulateScan(const tf::Pose &pose, double **data) {
  double yaw = tf::getYaw(pose.getRotation());
  auto yawTrans = utils::createTfFromXYTheta(pose.getOrigin().x(), pose.getOrigin().y(), yaw);

  for(int i = 0; i < scan_.ranges.size(); i++) {
    increment = scan_.angle_min + i * scan_.angle_increment;
    double theta = yaw + increment;
    pos = pose.getOrigin();

    while(true) {
      if(!utils::odomToCostmap(map_.info, pos, costmap)) {
        ldp->valid[i] = 0;
        ldp->readings[i] = -1;  // for invalid range
        break;
      }
      auto index = (size_t)(costmap.x() + costmap.y() * map_.info.width);
      if(map_.data[index] == 100) {
        auto distance = pos.distance(pose.getOrigin());
        if(distance > scan_.range_max) {
          ldp->valid[i] = 0;
          ldp->readings[i] = -1;
        } else {
          ldp->valid[i] = 1;
          ldp->readings[i] = distance;
        }
        break;
      } else if(map_.data[index] == -1) {  // unknown
        ldp->valid[i] = 0;
        ldp->readings[i] = -1;
        break;
      }
      pos.setX(pos.x() + map_.info.resolution * cos(theta));
      pos.setY(pos.y() + map_.info.resolution * sin(theta));
    }

    scan.ranges.push_back(ldp->readings[i]);

    ldp->theta[i] = increment;
    ldp->cluster[i] = -1;
  }

  std::vector<tf::Vector3> points;
  nav_msgs::GridCells grid;
  grid.header = scan_.header;
  grid.cell_width = grid.cell_height = map_.info.resolution;

  int r = int(scan_.range_max / map_.info.resolution);
  tf::Vector3 costmap;
  if(!utils::odomToCostmap(map_.info, pose.getOrigin(), costmap)) {
    return 0;
  }
  int r2 = r * r;
  int area = r2 << 2;
  int rr = r << 1;

  for (int i = 0; i < area; i++)
  {
    int tx = (i % rr) - r;
    int ty = (i / rr) - r;

    if (tx * tx + ty * ty <= r2) {
      int x = costmap.x() + tx, y = costmap.y() + ty;
      if(x < 0 || x >= map_.info.width || y < 0 || y >= map_.info.height) {
        continue;
      }
      auto index = (size_t)(x + y*map_.info.width);
      if(map_.data[index] == 100) {
        auto odom = yawTrans.inverse() * utils::costmapToOdom(map_.info, tf::Vector3(x, y, 0));
        points.push_back(odom);
        grid.cells.push_back(utils::vectorToPoint(odom));
      }
    }
  }

  *data = new double[2 * points.size()];
  for(int i = 0; i < points.size(); i++) {
    (*data)[i*2] = points[i].x();
    (*data)[i*2+1] = points[i].y();
  }

  scanPub_.publish(grid);
  return points.size();
}

*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_localization");
  MapLocalizationNode node;
  ros::spin();
  return 0;
}