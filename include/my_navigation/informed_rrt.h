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
 #include <nav_msgs/Path.h>
 #include <geometry_msgs/Pose2D.h>
 #include <nav_msgs/OccupancyGrid.h>
 #include <sensor_msgs/PointCloud.h>

 #include <Eigen/Dense>

 #include "node.h"

 using std::string;
 using namespace Eigen;

 #ifndef INFORMED_RRT_H
 #define INFORMED_RRT_H

 #define PI 3.14159265

 namespace informed_rrt {
     class InformedRrt : public nav_core::BaseGlobalPlanner {
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
         nav_msgs::OccupancyGrid ref_map_;
         ros::Subscriber sub_ref_map_;
         nav_msgs::OccupancyGrid ori_map_;
         ros::Subscriber sub_ori_map_;
         base_local_planner::WorldModel* world_model_;
         ros::Publisher potarr_pub_;

         void subRefMap(nav_msgs::OccupancyGrid map);
         void subOriMap(nav_msgs::OccupancyGrid map);

         geometry_msgs::Pose2D start_pose_;
         geometry_msgs::Pose2D goal_;
         vector<Node*> start_tree_;
         Node* joint_node_;
         vector<Node*> goal_tree_;
         nav_msgs::Path path_;
         ros::Publisher pub_path_;
         int pose_valid_threshold_;
         double step_;
         double trim_scope_;
         double ave_v_;
         double dist_of_two_node_;

         double footprintCost(double x_i, double y_i, double theta_i);
         int informedRrtSearch();
         geometry_msgs::Pose2D generateRandPoint();
         void mapToWorld(int mx, int my, double& wx, double& wy);
         void worldToMap(double wx, double wy, int& mx, int& my);
         bool obstacleCheck(double wx, double wy);
         void extendTheTree(geometry_msgs::Pose2D point);
         bool generateValidTreeNode(geometry_msgs::Pose2D point, vector<Node*>& tree);
         bool connectTwoNode(Node* node1, Node* Node2, bool connect_flag);
         bool connectTwoNode2(Node* node1, Node* Node2, bool connect_flag);
         bool findPathCheck(vector<Node*>& tree1, vector<Node*>& tree2);
         void getTheInitPath();
         double getPathLength();
         void trimTheTree(geometry_msgs::Pose2D point);

         ros::Publisher pub_tree_;
         ros::Publisher pub_node_;
         ros::Publisher pub_point_;
         ros::Publisher pub_path_2_;
         void displayTree();
         void displayTree2();
         void displayPoint(double x, double y);
     };
 };
 #endif