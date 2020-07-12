#include <pluginlib/class_list_macros.h>
#include "informed_rrt.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(informed_rrt::InformedRrt, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace informed_rrt 
{
    InformedRrt::InformedRrt()
    : costmap_ros_(NULL), initialized_(false)
    {}

    InformedRrt::InformedRrt(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false) {
        initialize(name, costmap_ros);
    }

    void InformedRrt::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if(!initialized_) {
            costmap_ = costmap_ros->getCostmap();
            //planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            //ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle private_nh;
            //pub_path_ = private_nh.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan", 1);
            pub_path_ = private_nh.advertise<nav_msgs::Path>("/own_path", 1);
            sub_ref_map_ = private_nh.subscribe("/move_base/global_costmap/costmap",1,&InformedRrt::subRefMap,this);
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            
            //world_model_ = new base_local_planner::CostmapModel(*costmap_); 

            initialized_ = true;

            path_.header.frame_id = costmap_ros->getGlobalFrameID();
            path_.poses.clear();
        }
        else {
            ROS_WARN("This planner has already been initialized... doing nothing");
        }
    }

    void InformedRrt::subRefMap(nav_msgs::OccupancyGrid map) {
        ref_map_ = map;
    }

    double InformedRrt::footprintCost(double x_i, double y_i, double theta_i) {
        if(!initialized_) {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return -1.0;
        }

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        //if we have no footprint... do nothing
        if(footprint.size() < 3)
            return -1.0;

        //check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);

        return footprint_cost;
    }

    bool InformedRrt::makePlan(const geometry_msgs::PoseStamped& start, 
                               const geometry_msgs::PoseStamped& goal, 
                               std::vector<geometry_msgs::PoseStamped>& plan) {
        start_pose_.x = start.pose.position.x;
        start_pose_.y = start.pose.position.y;
        start_pose_.theta = acos(2 * start.pose.orientation.w * start.pose.orientation.w - 1);
        start_pose_.theta = start.pose.orientation.w * start.pose.orientation.z < 0 ? -start_pose_.theta : start_pose_.theta;
        goal_.x = goal.pose.position.x;
        goal_.y = goal.pose.position.y;
        goal_.theta = acos(2 * goal.pose.orientation.w * goal.pose.orientation.w - 1);
        goal_.theta = goal.pose.orientation.w * goal.pose.orientation.z < 0 ? -goal_.theta : goal_.theta;
        cout << "start: " << start_pose_.x << ", " << start_pose_.y << ", " << start_pose_.theta << endl;
        cout << " goal: " << goal_.x << ", " << goal_.y << ", " << goal_.theta << endl;
        start_tree_ = new Node(start_pose_.x, start_pose_.y, start_pose_.theta);
        goal_tree_ = new Node(goal_.x, goal_.y, goal_.theta);

        ///////////////////////////////////
        // cout << ref_map_.header.frame_id << ", " << ref_map_.info.resolution << ", "
        //      << ref_map_.info.width << ", " << ref_map_.info.height << ", " << ref_map_.info.origin.position.x
        //      << ", " << ref_map_.info.origin.position.y << endl;
        ///////////////////////////////////
        int map_x, map_y;
        worldToMap(start_pose_.x, start_pose_.y, map_x, map_y);
        costmap_->setCost(map_x, map_y, costmap_2d::FREE_SPACE);
        cout << "obstacle check: " << obstacleCheck(goal_.x, goal_.y) << endl;
        // while(ros::ok()) {
        //     int stop_flag = informedRrtSearch();
        //     if(stop_flag == 0) {}
        //     else if(stop_flag == 1) {
        //         cout << "find the path, and pub the path." << endl;
        //         pub_path_.publish(path_);
        //         return true;
        //     }
        //     else if(stop_flag == 2) {
        //         cout << "can't find the path." << endl;
        //         return false;
        //     }
        //     ros::Duration(1).sleep();
        // }

        ros::Duration(1).sleep();
    }

    int InformedRrt::informedRrtSearch() {
        int ans = 0;
        geometry_msgs::Pose2D rand_point;
        rand_point = generateRandPoint();
        if(path_.poses.size() == 0) {}
        else {}
        return ans;
    }

    geometry_msgs::Pose2D InformedRrt::generateRandPoint() {}

    void InformedRrt::mapToWorld(int mx, int my, double& wx, double& wy) {
        wx = costmap_->getOriginX() + mx * costmap_->getResolution();
        wy = costmap_->getOriginY() + my * costmap_->getResolution();
    }

    void InformedRrt::worldToMap(double wx, double wy, int& mx, int& my) {
        unsigned int x, y;
        costmap_->worldToMap(wx, wy, x, y);
        mx = int(x);
        my = int(y);
    }

    bool InformedRrt::obstacleCheck(double wx, double wy) {
        unsigned int index;
        int mx, my;
        worldToMap(wx, wy, mx, my);
        index = mx + my * ref_map_.info.width;
        if(ref_map_.data[index] < 50) {
            return 0;
        }
        else {
            return 1;
        }
    }
};