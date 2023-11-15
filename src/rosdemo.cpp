#ifndef TEST_CYL
#define TEST_CYL

#include <iostream>
#include <vector>

#include "yaml-cpp/yaml.h"

#if OPEN3D == 1
  #include "Synchro.h"
#endif

#include "Cylinder.h"
#include "DataLoader.h"
#include "common_macro.hpp"
#include "cv_ext.h"

#include "ros/ros.h"
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/common/transforms.h>

using namespace std::chrono_literals;


YAML::Node sample_data = YAML::LoadFile(ros::package::getPath("trav_analysis_2") + std::string("/models/test.yaml"));
cv_ext::BasicTimer bt;

std::vector<Cylinder*> cyls;
std::vector<int> seqs;
int visualization_offset;
std::string dataset_mode;

bool already_written = false;
uint64_t runtime, tot_runtime=0;
double avg_runtime;

#if OPEN3D == 1
Synchro synchro(sample_data, true);
#endif

template<typename T>
void loadCyls(std::vector<T*> &cyls, YAML::Node &sample_data) {
  cyls.clear();
  int level;

  std::cout << "DATA: " << dataset_mode << std::endl;
  std::cout << "#######################################" << std::endl;
  for (level=0; ; level++) {
    auto cyl_s = std::string("cyl") + std::string(2 - MIN(2, std::to_string(level).length()), '0') + std::to_string(level);
    YAML::Node node = sample_data["general"][cyl_s.c_str()];
    if (!node) break;

    node["dataset"] = dataset_mode;
    node["load_path"] = sample_data["general"]["load_path"].as<std::string>();
    node["save_path"] = sample_data["general"]["save_path"].as<std::string>();
    
    T *back_cyl = (level>0) ? (cyls[level-1]) : nullptr;
    
    auto cyl = new Cylinder_SinglePLY(node, back_cyl, ExpMode::test); 
    cyl->printSummary();   
    cyls.push_back(cyl);

  }
  std::cout << "#######################################" << std::endl;

  if (!level) throw std::runtime_error(
                 std::string("\033[1;31mERROR\033[0m. please provide"
                  " at least a cylinder in yaml config file.\n"));
}


 DataLoader_ROS *dl_ros;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//PointCloud::Ptr ptc (new PointCloud);

void callback_handlePointCloud(const PointCloud::ConstPtr& msg) {
  ROS_INFO("I heard a point cloud");

  PointCloud::Ptr ptc (new PointCloud);

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  float theta = -M_PI/2; // The angle of rotation in radians
  transform_1 (0,0) = std::cos (theta);
  transform_1 (0,2) = sin(theta);
  transform_1 (2,0) = -sin (theta);
  transform_1 (2,2) = std::cos (theta);

  Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

  theta = M_PI/2; // The angle of rotation in radians
  transform_2 (0,0) = std::cos (theta);
  transform_2 (0,1) = -sin(theta);
  transform_2 (1,0) = sin (theta);
  transform_2 (1,1) = std::cos (theta);

  transform_1 = transform_1 * transform_2;

  pcl::transformPointCloud(*msg, *ptc, transform_1);

  dl_ros->setData(ptc);
  /*
  try{
    for (auto cyl : cyls) {
      cyl->OnlineRoutine(*dl_ros, cyl->level ? cyls[cyl->level-1] : nullptr);
    // runtime = cyl->OnlineRoutineProfile(dl_ply, cyl->level ? cyls[cyl->level-1] : nullptr);
    // tot_runtime += runtime;
    }

    synchro.addPointCloud(*dl_ros);
    synchro.addPolarGridPred(1, cyls[1]->grid);
  } catch (const cv::Exception e) {

  }
  */

}




int main (int argc, char** argv)
{

  ros::init(argc, argv, "traversability");
  ros::NodeHandle n;

  ROS_INFO("Starting traversability");

  ros::Subscriber sub = n.subscribe<PointCloud>("/visual/pointcloud", 10, callback_handlePointCloud);
  //ros::Publisher  pub = n.advertise<

  
  seqs                 = sample_data["general"]["split"]["test"].as<std::vector<int>>();
  visualization_offset = sample_data["general"]["vis_offset"].as<int>();
  dataset_mode         = sample_data["general"]["dataset"].as<std::string>();

  // Remap the load path of the yaml file
  std::string load_path;
  n.getParam("/traversability/load_path", load_path);
  //ROS_WARN("%s",load_path.c_str());

  sample_data["general"]["load_path"] = load_path;
   
  loadCyls(cyls, sample_data);

#if OPEN3D == 1
  synchro.resetViewFlag(); 
#endif

  dl_ros = new DataLoader_ROS(std::string("dummy"));

  ros::Rate loop_rate(16);

  while (ros::ok()) {
    ros::spinOnce();

    try{
      for (auto cyl : cyls) {
        cyl->OnlineRoutine(*dl_ros, cyl->level ? cyls[cyl->level-1] : nullptr);
        // runtime = cyl->OnlineRoutineProfile(dl_ply, cyl->level ? cyls[cyl->level-1] : nullptr);
        // tot_runtime += runtime;
      }
      synchro.addPointCloud(*dl_ros);
      synchro.addPolarGridPred(1, cyls[1]->grid);
    } catch (const cv::Exception e) {
      // pass;
    }


    loop_rate.sleep();
  }

  //ros::spin();


  /*
  // std::string single_cloud_path = "/media/fusy/Windows-SSD/Users/orasu/Downloads/LiDAR-GTA-V-1.2/LiDAR-GTA-V-1.2/samples/LiDAR - Traffic.ply";
  std::string single_cloud_path = "/home/fusy/repos/velodyne/";
  DataLoader_PLY dl_ply(single_cloud_path);

  for (int i=0; i<10; i++) {
  // while(true) {

    std::string idx_s = std::to_string(i);
    auto new_idx_s = std::string(6 - MIN(6, idx_s.length()), '0') + idx_s;

#if OPEN3D == 1
    synchro.reset();
#endif

    // dl_ply.readPLY(single_cloud_path);
    // dl_ply.readBin(single_cloud_path + new_idx_s + ".bin");
    dl_ply.readBin("/home/fusy/repos/pandaset_lidar/sweeps/" + new_idx_s + ".bin");
    //dl_nusc.readPredicted(0, sample_idx, sample_data);

    std::cout << "points " << dl_ply.points.size() << std::endl;
    std::cout << "labels " << dl_ply.labels.size() << std::endl;
    // std::cout << "pred_labels " << dl_ply.pred_labels.size() << std::endl;
    
    for (auto cyl : cyls) {
    // cyl->OnlineRoutine(dl, cyl->level ? cyls[cyl->level-1] : nullptr);
    runtime = cyl->OnlineRoutineProfile(dl_ply, cyl->level ? cyls[cyl->level-1] : nullptr);
    tot_runtime += runtime;
    }
    std::cout << " total latency: " << runtime << " ms" << std::endl;
    
#if OPEN3D == 1
    synchro.addPointCloud(dl_ply); // synchro.addPointCloudVoxeled(dl, 0.02f);
    synchro.addPolarGridPred(2, cyls[2]->grid);
    synchro.delay(visualization_offset);
#endif



//   for (size_t l=0; l<cyls.size(); l++)
//     cyls[l]->gmetric.print(
//         std::string("final lv ") + std::to_string(l) + std::string(": "), 
//         cyls[l]->tot_cells, 1);
  
}
#if OPEN3D == 1
  synchro.join();
#endif
//   for (auto cyl: cyls) free(cyl);
*/
  return 0;
}

#endif //TEST_CYL
