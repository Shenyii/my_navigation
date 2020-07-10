/** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <tf/tf.h>
 #include <tf2/convert.h>
 #include <tf2/utils.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

 using std::string;

 #ifndef INFORMED_RRT_H
 #define INFORMED_RRT_H

 namespace informed_rrt 
 {
     class InformedRrt : public nav_core::BaseGlobalPlanner 
     {
     public:
         InformedRrt();
         InformedRrt(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
         InformedRrt(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

         /** overridden classes from interface nav_core::BaseGlobalPlanner **/
         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
         void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

         bool makePlan(const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& plan);
         bool makePlan(const geometry_msgs::PoseStamped& start, 
                       const geometry_msgs::PoseStamped& goal, double tolerance, 
                       std::vector<geometry_msgs::PoseStamped>& plan);

     private:
         bool initialized_;
         costmap_2d::Costmap2DROS* costmap_ros_;
         double step_size_, min_dist_from_robot_;
         costmap_2d::Costmap2D* costmap_;
         base_local_planner::WorldModel* world_model_;
         double footprintCost(double x_i, double y_i, double theta_i);
     };
 };
 #endif