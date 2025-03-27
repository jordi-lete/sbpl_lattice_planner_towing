#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

// global representation
#include <nav_core/base_global_planner.h>

// TF stuff for simulation
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

// struct TrailerState {
//     double theta1;
//     double theta2;
// };

namespace sbpl_lattice_planner{

class SBPLLatticePlanner : public nav_core::BaseGlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlanner();

  
  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~SBPLLatticePlanner(){};

private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  unsigned char computeCircumscribedCost();

  static void transformFootprintToEdges(const geometry_msgs::Pose& robot_pose,
                                        const std::vector<geometry_msgs::Point>& footprint,
                                        std::vector<geometry_msgs::Point>& out_footprint);

  void getFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t>& sbpl_path, const std::string& path_frame_id,
                        visualization_msgs::Marker& ma);

  void getTrailerFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t>& sbpl_trailer_path, const std::string& path_frame_id,
                        visualization_msgs::Marker& ma);

  void trailerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void getTrailerPose(TrailerState& trailer, const geometry_msgs::PoseStamped& base_pose);

  bool has_trailer_pose_;
  geometry_msgs::PoseStamped last_trailer_pose_;
  bool initialized_;

  SBPLPlanner* planner_;

  bool using_custom_envirment_;
  EnvironmentNAVXYTHETALAT* navxythetalat_env_;
  EnvironmentXXXLAT* xxx_env_;
  // EnvironmentXXXLAT* env_;
  // DiscreteSpaceInformation* env_base;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char circumscribed_cost_;
  unsigned char sbpl_cost_multiplier_;

  bool publish_footprint_path_;
  int visualizer_skip_poses_;

  bool allow_unknown_;

  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  std::vector<geometry_msgs::Point> footprint_;
  double trailer_width_;
  double trailer_length_;
  std::vector<geometry_msgs::Point> trailer_footprint_;
  unsigned int current_env_width_;
  unsigned int current_env_height_;

  ros::Publisher plan_pub_;
  ros::Publisher stats_publisher_;
  ros::Publisher sbpl_plan_footprint_pub_;
  ros::Publisher sbpl_plan_trailer_footprint_pub_;
  ros::Subscriber trailer_pose_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
};

#endif

