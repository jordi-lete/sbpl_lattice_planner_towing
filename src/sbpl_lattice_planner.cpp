/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Mike Phillips
*********************************************************************/

#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Path.h>
#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>

#include <costmap_2d/inflation_layer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// For trailer tf
#include <angles/angles.h>
#include <ros/console.h>

using namespace std;
using namespace ros;


PLUGINLIB_EXPORT_CLASS(sbpl_lattice_planner::SBPLLatticePlanner, nav_core::BaseGlobalPlanner)

namespace geometry_msgs {
  bool operator== (const Point &p1, const Point &p2)
  {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
  }
}

namespace sbpl_lattice_planner{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : navxythetalat_env_(env), xxx_env_(NULL), changedcellsV_(changedcellsV), using_custom_envirment_(false) {
    }
    LatticeSCQ(EnvironmentXXXLAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : navxythetalat_env_(NULL), xxx_env_(env), changedcellsV_(changedcellsV), using_custom_envirment_(true) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        if (using_custom_envirment_) {
          xxx_env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
        } else {
          navxythetalat_env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
        }
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        if (using_custom_envirment_) {
          xxx_env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
        } else{
          navxythetalat_env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
        }
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * navxythetalat_env_;
    EnvironmentXXXLAT * xxx_env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
    bool using_custom_envirment_;
};

SBPLLatticePlanner::SBPLLatticePlanner()
  : initialized_(false), costmap_ros_(NULL){
}

SBPLLatticePlanner::SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
  : initialized_(false), costmap_ros_(NULL){
  ROS_DEBUG("Constructor is running!!!!!!!!!!!!!!!!!!!!!1");
  initialize(name, costmap_ros);
}

    
void SBPLLatticePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
      ros::console::notifyLoggerLevelsChanged();
    }
    ROS_DEBUG("Logging level set to debug");
    ros::NodeHandle private_nh("~/"+name);

    ROS_DEBUG("Name is %s", name.c_str());

    private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
    private_nh.param("allocated_time", allocated_time_, 10.0);
    private_nh.param("initial_epsilon",initial_epsilon_,3.0);
    private_nh.param("environment_type", environment_type_, string("XYThetaLattice"));
    private_nh.param("forward_search", forward_search_, bool(false));
    private_nh.param("primitive_filename",primitive_filename_,string(""));
    private_nh.param("force_scratch_limit",force_scratch_limit_,500);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    int lethal_obstacle;
    private_nh.param("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);

    private_nh.param("publish_footprint_path", publish_footprint_path_, bool(true));
    private_nh.param<int>("visualizer_skip_poses", visualizer_skip_poses_, 5);

    private_nh.param("allow_unknown", allow_unknown_, bool(true));

    name_ = name;
    costmap_ros_ = costmap_ros;

    footprint_ = costmap_ros_->getRobotFootprint();
    std::vector<geometry_msgs::Point> trailer_footprint;
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint_.size());
    for (size_t ii(0); ii < footprint_.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint_[ii].x;
      pt.y = footprint_[ii].y;
      perimeterptsV.push_back(pt);
    }
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);

    using_custom_envirment_ = false;
    navxythetalat_env_ = nullptr;
    xxx_env_ = nullptr;
    bool ret;

    if (environment_type_ == "XYThetaLattice"){
        ROS_INFO("Using a 3D costmap for theta lattice\n");
        navxythetalat_env_ = new EnvironmentNAVXYTHETALAT();
        using_custom_envirment_ = false;
        circumscribed_cost_ = computeCircumscribedCost();
        if (circumscribed_cost_ == 0) {
          // Unfortunately, the inflation_radius is not taken into account by
          // inflation_layer->computeCost(). If inflation_radius is smaller than
          // the circumscribed radius, SBPL will ignore some obstacles, but we
          // cannot detect this problem. If the cost_scaling_factor is too large,
          // SBPL won't run into obstacles, but will always perform an expensive
          // footprint check, no matter how far the nearest obstacle is.
          ROS_WARN("The costmap value at the robot's circumscribed radius (%f m) is 0.", costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
          ROS_WARN("SBPL performance will suffer.");
          ROS_WARN("Please decrease the costmap's cost_scaling_factor.");
        }
        if(!navxythetalat_env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
            ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
            exit(1);
        }
        if(!navxythetalat_env_->SetEnvParameter("cost_possibly_circumscribed_thresh", circumscribed_cost_)){
            ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
            exit(1);
        }
        try{
            ret = navxythetalat_env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                        costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                        0, // mapdata
                                        0, 0, 0, // start (x, y, theta, t)
                                        0, 0, 0, // goal (x, y, theta)
                                        0, 0, 0, //goal tolerance
                                        perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                        timetoturn45degsinplace_secs, obst_cost_thresh,
                                        primitive_filename_.c_str());
            current_env_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
            current_env_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
        }
        catch(SBPL_Exception *e){
            ROS_ERROR("SBPL encountered a fatal exception: %s", e->what());
            ret = false;
        }
        if(!ret){
            ROS_ERROR("SBPL initialization failed!");
            exit(1);
        }
    }
    else if(environment_type_ == "XXX") {
        ROS_INFO("Using a custom environment: XXX\n");
        xxx_env_ = new EnvironmentXXXLAT();
        using_custom_envirment_ = true;
        circumscribed_cost_ = computeCircumscribedCost();
        if (circumscribed_cost_ == 0) {
          // Unfortunately, the inflation_radius is not taken into account by
          // inflation_layer->computeCost(). If inflation_radius is smaller than
          // the circumscribed radius, SBPL will ignore some obstacles, but we
          // cannot detect this problem. If the cost_scaling_factor is too large,
          // SBPL won't run into obstacles, but will always perform an expensive
          // footprint check, no matter how far the nearest obstacle is.
          ROS_WARN("The costmap value at the robot's circumscribed radius (%f m) is 0.", costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
          ROS_WARN("SBPL performance will suffer.");
          ROS_WARN("Please decrease the costmap's cost_scaling_factor.");
        }
        if(!xxx_env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
            ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
            exit(1);
        }
        if(!xxx_env_->SetEnvParameter("cost_possibly_circumscribed_thresh", circumscribed_cost_)){
            ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
            exit(1);
        }

        private_nh.param("trailer_length", trailer_length_, 0.0);
        private_nh.param("trailer_width", trailer_width_, 0.0);

        trailer_footprint.clear();
        geometry_msgs::Point pt;
        pt.x = trailer_length_/2;
        pt.y = trailer_width_/2;
        trailer_footprint.push_back(pt);
        pt.y = -trailer_width_/2;
        trailer_footprint.push_back(pt);
        pt.x = -trailer_length_/2;
        trailer_footprint.push_back(pt);
        pt.y = trailer_width_/2;
        trailer_footprint.push_back(pt);

        vector<sbpl_2Dpt_t> trailer_perimeterptsV;
        trailer_perimeterptsV.reserve(trailer_footprint.size());
        for (size_t ii(0); ii < trailer_footprint.size(); ++ii) {
            sbpl_2Dpt_t pt;
            pt.x = trailer_footprint[ii].x;
            pt.y = trailer_footprint[ii].y;
            trailer_perimeterptsV.push_back(pt);
        }
        trailer_footprint_ = trailer_footprint;
        try{
            ret = xxx_env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                        costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                        0, // mapdata
                                        0, 0, 0, 0, 0, // start (x, y, theta, t)
                                        0, 0, 0, // goal (x, y, theta)
                                        0.5, 0.5, 6.0, //goal tolerance (m, m, rad)
                                        perimeterptsV, trailer_perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                        timetoturn45degsinplace_secs, obst_cost_thresh,
                                        primitive_filename_.c_str());
            current_env_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
            current_env_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
        }
        catch(SBPL_Exception *e){
            ROS_ERROR("SBPL encountered a fatal exception: %s", e->what());
            ret = false;
        }
        if(!ret){
            ROS_ERROR("SBPL initialization failed!");
            exit(1);
        }
    }
    else{
      ROS_ERROR("Unsupported environment type: %s\n", environment_type_.c_str());
      exit(1);
    }

    for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
        if (using_custom_envirment_) {
            xxx_env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));
        } else {
            navxythetalat_env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));
        }
    if (using_custom_envirment_){
        if ("ARAPlanner" == planner_type_){
        ROS_INFO("Planning with ARA*");
        planner_ = new ARAPlanner(xxx_env_, forward_search_);
        }
        else if ("ADPlanner" == planner_type_){
        ROS_INFO("Planning with AD*");
        planner_ = new ADPlanner(xxx_env_, forward_search_);
        }
        else{
        ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
        exit(1);
        }
    } else {
        if ("ARAPlanner" == planner_type_){
        ROS_INFO("Planning with ARA*");
        planner_ = new ARAPlanner(navxythetalat_env_, forward_search_);
        }
        else if ("ADPlanner" == planner_type_){
        ROS_INFO("Planning with AD*");
        planner_ = new ADPlanner(navxythetalat_env_, forward_search_);
        }
        else{
        ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
        exit(1);
        }
    }
    ROS_INFO("[sbpl_lattice_planner] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = private_nh.advertise<sbpl_lattice_planner::SBPLLatticePlannerStats>("sbpl_lattice_planner_stats", 1);
    sbpl_plan_footprint_pub_ = private_nh.advertise<visualization_msgs::Marker>("footprint_markers", 1);
    has_trailer_pose_ = false;
    if (using_custom_envirment_){
        trailer_pose_sub_ = private_nh.subscribe("trailer_pose", 1, &SBPLLatticePlanner::trailerPoseCallback, this);
        tf_listener_= std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    }
    sbpl_plan_trailer_footprint_pub_ = private_nh.advertise<visualization_msgs::Marker>("trailer_footprint_markers", 1);
    initialized_ = true;
  }
}
  
//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char SBPLLatticePlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE || (!allow_unknown_ && newcost == costmap_2d::NO_INFORMATION))
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else {
    unsigned char sbpl_cost = newcost / sbpl_cost_multiplier_;
    if (sbpl_cost == 0)
      sbpl_cost = 1;
    return sbpl_cost;
  }
}

void SBPLLatticePlanner::publishStats(int solution_cost, int solution_size, 
                                      const geometry_msgs::PoseStamped& start, 
                                      const geometry_msgs::PoseStamped& goal){
  // Fill up statistics and publish
  sbpl_lattice_planner::SBPLLatticePlannerStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = planner_->get_n_expands();
  stats.allocated_time = allocated_time_;

  stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
  stats.actual_time = planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
  stats.final_epsilon = planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;
  stats_publisher_.publish(stats);
}

unsigned char SBPLLatticePlanner::computeCircumscribedCost() {
  unsigned char result = 0;

  if (!costmap_ros_) {
    ROS_ERROR("Costmap is not initialized");
    return 0;
  }

  // check if the costmap has an inflation layer
  for(std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
      layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
      ++layer) {
    boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer) continue;

    result = costMapCostToSBPLCost(inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() / costmap_ros_->getCostmap()->getResolution()));
  }
  return result;
}

void SBPLLatticePlanner::trailerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  last_trailer_pose_ = *msg;
  has_trailer_pose_ = true;
}

void SBPLLatticePlanner::getTrailerPose(TrailerState& trailer, const geometry_msgs::PoseStamped& base_pose) {
  if (has_trailer_pose_ && (ros::Time::now() - last_trailer_pose_.header.stamp) < ros::Duration(1.0)) {
    ROS_INFO("Using trailer localisation for start pose");
    trailer.theta1 = 2 * atan2(last_trailer_pose_.pose.orientation.z, last_trailer_pose_.pose.orientation.w);
    // need to change these for the actual message type
    trailer.theta2 = 2 * atan2(last_trailer_pose_.pose.orientation.z, last_trailer_pose_.pose.orientation.w);
    return;
  }
  try {
    geometry_msgs::TransformStamped theta1_transform = tf_buffer_.lookupTransform("map", "connector_1", ros::Time(0));
    geometry_msgs::TransformStamped theta2_transform = tf_buffer_.lookupTransform("map", "trailer_1", ros::Time(0));
    trailer.theta1 = tf2::getYaw(theta1_transform.transform.rotation);
    trailer.theta2 = tf2::getYaw(theta2_transform.transform.rotation);
    ROS_INFO("Using gazebo tf for start pose: %f %f", trailer.theta1, trailer.theta2);
    trailer.theta1 = angles::normalize_angle(trailer.theta1);
    trailer.theta2 = angles::normalize_angle(trailer.theta2);
    ROS_INFO("Using gazebo tf for start pose: %f %f", trailer.theta1, trailer.theta2);
  } 
  catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "Could not get trailer transforms: %s. Using default angles.", ex.what());
    double theta = 2 * atan2(base_pose.pose.orientation.z, base_pose.pose.orientation.w);
    trailer.theta1 = theta;
    trailer.theta2 = theta;
  }
}

bool SBPLLatticePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
  if(!initialized_){
    ROS_ERROR("Global planner is not initialized");
    return false;
  }

  bool do_init = false;
  if (current_env_width_ != costmap_ros_->getCostmap()->getSizeInCellsX() ||
      current_env_height_ != costmap_ros_->getCostmap()->getSizeInCellsY()) {
    ROS_INFO("Costmap dimensions have changed from (%d x %d) to (%d x %d), reinitializing sbpl_lattice_planner.",
             current_env_width_, current_env_height_,
             costmap_ros_->getCostmap()->getSizeInCellsX(), costmap_ros_->getCostmap()->getSizeInCellsY());
    do_init = true;
  }
  else if (footprint_ != costmap_ros_->getRobotFootprint()) {
    ROS_INFO("Robot footprint has changed, reinitializing sbpl_lattice_planner.");
    do_init = true;
  }
  else if (circumscribed_cost_ != computeCircumscribedCost()) {
    ROS_INFO("Cost at circumscribed radius has changed, reinitializing sbpl_lattice_planner.");
    do_init = true;
  }

  if (do_init) {
    initialized_ = false;
    delete planner_;
    planner_ = NULL;
    if (using_custom_envirment_) {
        delete xxx_env_;
        xxx_env_ = NULL;
    } else {
        delete navxythetalat_env_;
        navxythetalat_env_ = NULL;
    }
    ROS_DEBUG("Reinitializing planner....");
    initialize(name_, costmap_ros_);
  }

  plan.clear();

  ROS_INFO("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);
  int ret;

  if (using_custom_envirment_) {
    TrailerState startTrailer;
    getTrailerPose(startTrailer, start);
    try{
        ret = xxx_env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
        xxx_env_->SetTrailerStart(startTrailer.theta1, startTrailer.theta2);
        if(ret < 0 || planner_->set_start(ret) == 0){
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    }
    catch(SBPL_Exception *e){
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }
    try{
        ret = xxx_env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
        if(ret < 0 || planner_->set_goal(ret) == 0){
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
        ROS_INFO("SBPL goal state is set\n");
    }
    catch(SBPL_Exception *e){
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }
  } else {
    try{
        ret = navxythetalat_env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
        if(ret < 0 || planner_->set_start(ret) == 0){
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    }
    catch(SBPL_Exception *e){
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }
    try{
        ret = navxythetalat_env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
        if(ret < 0 || planner_->set_goal(ret) == 0){
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
        ROS_INFO("SBPL goal state is set\n");
    }
    catch(SBPL_Exception *e){
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }
  }
  
  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;

  for(unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++) {
    for(unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {
      unsigned char oldCost;
      if (using_custom_envirment_) {
        oldCost = xxx_env_->GetMapCost(ix,iy);
      } else {
        oldCost = navxythetalat_env_->GetMapCost(ix,iy);
      }
      unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));

      if(oldCost == newCost) continue;

      allCount++;

      //first case - off cell goes on

      if((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

      if((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      if (using_custom_envirment_) {
        xxx_env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));
      } else {
        navxythetalat_env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));
      }
      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

  try{
    if(!changedcellsV.empty()){
      StateChangeQuery* scq;
      if (using_custom_envirment_) {
        scq = new LatticeSCQ(xxx_env_, changedcellsV);
      } else {
        scq = new LatticeSCQ(navxythetalat_env_, changedcellsV);
      }
      planner_->costs_changed(*scq);
      delete scq;
    }

    if(allCount > force_scratch_limit_)
      planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception *e){
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  //setting planner parameters
  ROS_INFO("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(false);

  ROS_INFO("[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if (solution_stateIDs.empty())
      ROS_INFO("No states in solution path");
    else {
      ROS_INFO("Solution path contains %lu states", solution_stateIDs.size());
    }
    if(ret)
      ROS_DEBUG("Solution is found\n");
    else{
      ROS_INFO("Solution not found\n");
      publishStats(solution_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception *e){
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_INFO("size of solution=%d", (int)solution_stateIDs.size());

  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_trailer_path;
  try{
    if (using_custom_envirment_) {
      xxx_env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path, &sbpl_trailer_path);
    } else {
      navxythetalat_env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
    }
  }
  catch(SBPL_Exception *e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  // if the plan has zero points, add a single point to make move_base happy
  if( sbpl_path.size() == 0 ) {
    EnvNAVXYTHETALAT3Dpt_t s(
        start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
        start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(),
        theta_start);
    sbpl_path.push_back(s);
  }

  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  if (publish_footprint_path_)
  {
    visualization_msgs::Marker sbpl_plan_footprint;
    getFootprintList(sbpl_path, costmap_ros_->getGlobalFrameID(), sbpl_plan_footprint);
    sbpl_plan_footprint_pub_.publish(sbpl_plan_footprint);
    visualization_msgs::Marker sbpl_plan_trailer_footprint;
    if (using_custom_envirment_) {
        getTrailerFootprintList(sbpl_trailer_path, costmap_ros_->getGlobalFrameID(), sbpl_plan_trailer_footprint);
        sbpl_plan_trailer_footprint_pub_.publish(sbpl_plan_trailer_footprint);
    }
  }

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    pose.pose.position.z = start.pose.position.z;

    tf2::Quaternion temp;
    temp.setRPY(0,0,sbpl_path[i].theta);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i] = plan[i];
  }
  plan_pub_.publish(gui_path);
  publishStats(solution_cost, sbpl_path.size(), start, goal);

  return true;
}

void SBPLLatticePlanner::getFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t>& sbpl_path,
                                          const std::string& path_frame_id, visualization_msgs::Marker& ma)
{
  ma.header.frame_id = path_frame_id;
  ma.header.stamp = ros::Time();
  ma.ns = "sbpl_robot_footprint";
  ma.id = 0;
  ma.type = visualization_msgs::Marker::LINE_LIST;
  ma.action = visualization_msgs::Marker::ADD;
  ma.scale.x = 0.05;
  ma.color.a = 1.0;
  ma.color.r = 0.0;
  ma.color.g = 0.0;
  ma.color.b = 1.0;
  ma.pose.orientation.w = 1.0;

  for (unsigned int i = 0; i < sbpl_path.size(); i = i + visualizer_skip_poses_)
  {
    std::vector<geometry_msgs::Point> transformed_rfp;
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    ;
    robot_pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    ;
    robot_pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, sbpl_path[i].theta);
    tf::quaternionTFToMsg(quat, robot_pose.orientation);
    transformFootprintToEdges(robot_pose, footprint_, transformed_rfp);

    for (auto & point : transformed_rfp)
      ma.points.push_back(point);
  }
}

void SBPLLatticePlanner::getTrailerFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t>& sbpl_trailer_path,
                                          const std::string& path_frame_id, visualization_msgs::Marker& ma)
{
  ma.header.frame_id = path_frame_id;
  ma.header.stamp = ros::Time();
  ma.ns = "sbpl_trailer_footprint";
  ma.id = 0;
  ma.type = visualization_msgs::Marker::LINE_LIST;
  ma.action = visualization_msgs::Marker::ADD;
  ma.scale.x = 0.05;
  ma.color.a = 1.0;
  ma.color.r = 1.0;
  ma.color.g = 1.0;
  ma.color.b = 0.0;
  ma.pose.orientation.w = 1.0;

  for (unsigned int i = 0; i < sbpl_trailer_path.size(); i++)// i = i + visualizer_skip_poses_)
  {
    std::vector<geometry_msgs::Point> transformed_tfp;
    geometry_msgs::Pose trailer_pose;
    trailer_pose.position.x = sbpl_trailer_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    ;
    trailer_pose.position.y = sbpl_trailer_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    ;
    trailer_pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, sbpl_trailer_path[i].theta);
    tf::quaternionTFToMsg(quat, trailer_pose.orientation);
    transformFootprintToEdges(trailer_pose, trailer_footprint_, transformed_tfp);

    for (auto & point : transformed_tfp)
      ma.points.push_back(point);
  }
}

void SBPLLatticePlanner::transformFootprintToEdges(const geometry_msgs::Pose& robot_pose,
                                                   const std::vector<geometry_msgs::Point>& footprint,
                                                   std::vector<geometry_msgs::Point>& out_footprint)
{
  out_footprint.resize(2 * footprint.size());
  double yaw = tf::getYaw(robot_pose.orientation);
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    out_footprint[2 * i].x = robot_pose.position.x + cos(yaw) * footprint[i].x - sin(yaw) * footprint[i].y;
    out_footprint[2 * i].y = robot_pose.position.y + sin(yaw) * footprint[i].x + cos(yaw) * footprint[i].y;
    if (i == 0)
    {
      out_footprint.back().x = out_footprint[i].x;
      out_footprint.back().y = out_footprint[i].y;
    }
    else
    {
      out_footprint[2 * i - 1].x = out_footprint[2 * i].x;
      out_footprint[2 * i - 1].y = out_footprint[2 * i].y;
    }
  }
}
};
