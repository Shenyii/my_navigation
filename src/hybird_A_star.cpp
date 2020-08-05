#include "hybird_A_star.h"

HybirdAStar::HybirdAStar()
: start_x_(0), start_y_(0), start_theta_(0) {
    sub_ori_map_ = nh_.subscribe("/map", 2, &HybirdAStar::subOriMap, this);
    sub_start_pose_ = nh_.subscribe("/initialpose", 2, &HybirdAStar::subStartPose, this);

    while(ros::ok()) {
        ros::spin();
    }

    ///////////////////////
    int mx;
    int my;
    worldToMap(-6, -5, mx, my);
    cout << mx << ", " << my << endl;
    ///////////////////////
}

HybirdAStar::~HybirdAStar() {}

void HybirdAStar::subOriMap(nav_msgs::OccupancyGrid map) {
    ori_map_ = map;
    cout << "get the origin map." << endl;
}

void HybirdAStar::subStartPose(geometry_msgs::PoseWithCovarianceStamped start_pose) {
    start_x_ = start_pose.pose.pose.position.x;
    start_y_ = start_pose.pose.pose.position.y;
    start_theta_ = acos(2 * pow(start_pose.pose.pose.orientation.w, 2) - 1);
    start_theta_ = start_pose.pose.pose.orientation.w * start_pose.pose.pose.orientation.z < 0 ? - start_theta_ : start_theta_;
    //cout << start_x_ << ", " << start_y_ << ", " << start_theta_ << endl;
}

void HybirdAStar::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

int main(int argc, char** argv) {
    cout << "begin the program." << endl;
    ros::init(argc, argv, "hybird_A_star");
    HybirdAStar test;
    return 0;
}