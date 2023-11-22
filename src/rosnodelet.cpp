#ifndef TEST_CYL
#define TEST_CYL

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "yaml-cpp/yaml.h"

#if SYNCHRO_A == 1
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
//#include <std_msgs/String.h>
#include <proximity_grid/ProximityGridMsg.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/common/transforms.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

using namespace std::chrono_literals;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace traversability {
  class Trav : public nodelet::Nodelet
  {
      public:
        Trav() {
          this->new_data = false;
          this->sample_data = YAML::LoadFile(ros::package::getPath("trav_analysis_2") + std::string("/models/test.yaml"));

          #if SYNCHRO_A == 1
            this->synchro = new Synchro(this->sample_data, true);
          #endif 
        }
        virtual ~Trav() {}

        virtual void onInit() {
          ros::NodeHandle& n  = getNodeHandle();
          ros::NodeHandle& ph = getPrivateNodeHandle();
          sub_ = n.subscribe<PointCloud>("/visual/pointcloud", 10, &Trav::callback_handlePointCloud, this);

          // Here I will put the callback for update the data
          timer_ = n.createTimer(ros::Duration(1.0/16.0), boost::bind(& Trav::timer_callback, this, _1));
          pub_ = n.advertise<proximity_grid::ProximityGridMsg>("trav", 1);

          this->seqs                 = this->sample_data["general"]["split"]["test"].as<std::vector<int>>();
          this->visualization_offset = this->sample_data["general"]["vis_offset"].as<int>();
          this->dataset_mode         = this->sample_data["general"]["dataset"].as<std::string>();

          this->yaw_steps = this->sample_data["general"]["cyl01"]["yaw_steps"].as<int>();
          this->steps_num = this->sample_data["general"]["cyl01"]["steps_num"].as<int>();

          this->grid = Eigen::MatrixXf::Zero(this->steps_num, this->yaw_steps);

          this->outcome_probability_ = proximity_grid::ProximityGridMsg(); 
          // TODO: set these as parameters
          this->outcome_probability_.header.frame_id = "map";
          this->outcome_probability_.range_min = this->sample_data["general"]["cyl01"]["min_radius"].as<float>();
          this->outcome_probability_.range_max = this->sample_data["general"]["cyl01"]["max_radius"].as<float>();
          this->outcome_probability_.angle_min = -M_PI/2;
          this->outcome_probability_.angle_max =  M_PI/2;
          this->outcome_probability_.angle_increment = M_PI/this->yaw_steps;

          // Remap the load path of the yaml file
          std::string param_name;
          if (n.searchParam("traversability/load_path", param_name))
          {
            std::string load_path;
            
            ph.getParam(param_name, load_path);
            ROS_WARN("%s",load_path.c_str());

            this->sample_data["general"]["load_path"] = load_path;
          }
          
          // Load the svm model
          loadCyls(this->cyls, this->sample_data);

          #if SYNCHRO_A == 1
            this->synchro->resetViewFlag(); 
          #endif

          this->dl_ros = new DataLoader_ROS(std::string("dummy"));
          
        }

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
      
      private:
        void callback_handlePointCloud(const PointCloud::ConstPtr& msg) {
          PointCloud::Ptr ptc (new PointCloud);
          
          pcl::transformPointCloud(*msg, *ptc, this->getTransform());

          dl_ros->setData(ptc);

          new_data = true;
        }

        Eigen::Matrix4f getTransform() {
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

          return transform_1 * transform_2;
          
        }

        void timer_callback(const ros::TimerEvent& event){
          if( this->new_data ){

            try{
              for (auto cyl : cyls) {
                cyl->OnlineRoutine(*dl_ros, cyl->level ? cyls[cyl->level-1] : nullptr);
              }
              
              #if SYNCHRO_A == 1
                synchro->addPointCloud(*dl_ros);
                synchro->addPolarGridPred(1, cyls[1]->grid); // <-- this is the message
              #endif

              // This vector represents the grid matrix where the information is stored as follows
              // grid = [ [yaw_0, z_0] ... [yaw_0, z_n], [yaw_1, z_0] ... [yaw_1, z_n], ...]

              // TODO: remove these in favor of a eigen maps
              for ( int idx_col = 0; idx_col < this->yaw_steps; idx_col++ ) {
                for ( int idx_row = 0; idx_row < this->steps_num; idx_row++ ) {
                  if ( cyls[1]->grid[idx_row + idx_col * this->steps_num].predicted_label != 2 )
                    this->grid(idx_row, idx_col) = (cyls[1]->grid[idx_row + idx_col * this->steps_num].predicted_probability + 1) / 2;
                  else
                    this->grid(idx_row, idx_col) = 1; // 1 -> map the unknown cells as traversable
                                                      // 0 -> map the unknown cells as not traversable
                                                      // TODO: integrate this information as a rosparam
                }
              }
              

            } catch (const cv::Exception e) {
              // TODO: set a decayed timer to reset the grid if there is no data for a lot of time
              ; // pass
            }
            
            Eigen::VectorXf colSums = this->grid.colwise().prod();
            std::vector<float> compatible_data(colSums.data(), colSums.data() + colSums.rows());

            // Rotate the data by 45 degrees, to compensate previous rotation
            int rotation = (M_PI / 4) / this->outcome_probability_.angle_increment + 1;
            std::rotate(compatible_data.begin(), compatible_data.begin()+rotation, compatible_data.end());

            this->outcome_probability_.ranges = compatible_data;
            this->outcome_probability_.header.stamp = ros::Time::now();

            pub_.publish(this->outcome_probability_);

            this->new_data = false;
          }
        }

        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::Timer timer_;
        bool new_data;
        YAML::Node sample_data;

        proximity_grid::ProximityGridMsg outcome_probability_;


        DataLoader_ROS *dl_ros;
        cv_ext::BasicTimer bt;
        std::vector<Cylinder*> cyls;
        std::vector<int> seqs;
        int visualization_offset;
        std::string dataset_mode;

        Eigen::MatrixXf grid;

        int yaw_steps, steps_num;

        bool already_written = false;
        uint64_t runtime, tot_runtime=0;
        double avg_runtime;

        #if SYNCHRO_A == 1
          Synchro* synchro;
        #endif

        
  };

  PLUGINLIB_EXPORT_CLASS(traversability::Trav, nodelet::Nodelet);

}

#endif //TEST_CYL
