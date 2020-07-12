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
    {
        pub_path_ = n_.advertise<nav_msgs::Path>("plan", 1);
    }

    InformedRrt::InformedRrt(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false)
    {
        pub_path_ = n_.advertise<nav_msgs::Path>("plan", 1);
    }

    bool InformedRrt::makePlan(const geometry_msgs::PoseStamped& start, 
                               const geometry_msgs::PoseStamped& goal, 
                               std::vector<geometry_msgs::PoseStamped>& plan)
    {
        cout << "pub my path." << endl;
        nav_msgs::Path gui_path;
        gui_path.poses.resize(100);
        gui_path.header.frame_id = "map";
        for(int i = 0; i < gui_path.poses.size(); i++)
        {
            gui_path.poses[i].pose.position.y = -0.5;
            gui_path.poses[i].pose.position.x = -2 + 0.1 * i;
        }
        pub_path_.publish(gui_path);
        return true;
    }
};