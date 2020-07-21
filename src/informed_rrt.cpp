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
            sub_ori_map_ = private_nh.subscribe("/map",1,&InformedRrt::subOriMap,this);
            pose_valid_threshold_ = 90;
            step_ = 0.15;
            trim_scope_ = 0.5;
            ave_v_ = 1.0;
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            
            //world_model_ = new base_local_planner::CostmapModel(*costmap_); 

            pub_path_ = private_nh.advertise<nav_msgs::Path>("/own_path", 2);
            path_.header.frame_id = "map";

            pub_tree_ = private_nh.advertise<sensor_msgs::PointCloud>("/tree", 3);
            pub_node_ = private_nh.advertise<sensor_msgs::PointCloud>("/node", 3);
            pub_point_ = private_nh.advertise<sensor_msgs::PointCloud>("/point", 3);

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

    void InformedRrt::subOriMap(nav_msgs::OccupancyGrid map) {
        ori_map_ = map;
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
        // start_tree_.push_back(new Node(start_pose_.x, start_pose_.y, start_pose_.theta));
        // goal_tree_.push_back(new Node(goal_.x, goal_.y, goal_.theta));
        // start_tree_[0]->dist_to_root_ = 0;
        // goal_tree_[0]->dist_to_root_ = 0;

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

        ////////////////////////////////
        while(ros::ok()) {
            start_tree_.push_back(new Node(-2, 0, 0));
            goal_tree_.push_back(new Node(1, 2, PI / 2));
            start_tree_[0]->dist_to_root_ = 0;
            goal_tree_[0]->dist_to_root_ = 0;
            for(int i = 0; i < 200; i++) {
                informedRrtSearch();
                displayTree();
            }
            ros::Duration(1).sleep();
            getchar();
            for(int i = 0; i < start_tree_.size(); i++) {
                delete start_tree_[i];
            }
            for(int i = 0; i < goal_tree_.size(); i++) {
                delete goal_tree_[i];
            }
            //delete joint_node_;
            start_tree_.clear();
            goal_tree_.clear();
            path_.poses.clear();
            
            cout << endl << endl;
        }
        ////////////////////////////////

        for(int i = 0; i < start_tree_.size(); i++) {
            delete start_tree_[i];
        }
        for(int i = 0; i < goal_tree_.size(); i++) {
            delete goal_tree_[i];
        }
        //delete joint_node_;
        start_tree_.clear();
        goal_tree_.clear();
        path_.poses.clear();
    }

    int InformedRrt::informedRrtSearch() {
        int ans = 0;
        geometry_msgs::Pose2D rand_point;
        rand_point = generateRandPoint();
        if(path_.poses.size() == 0) {
            extendTheTree(rand_point);
        }
        else {
            double search_range = sqrt(pow(rand_point.x - start_tree_[0]->x_, 2) + pow(rand_point.y - start_tree_[0]->y_, 2)) + 
                                  sqrt(pow(rand_point.x - goal_tree_[0]->x_, 2) + pow(rand_point.y - goal_tree_[0]->y_, 2));
            while(search_range > getPathLength()) {
                rand_point = generateRandPoint();
                //displayPoint(rand_point.x, rand_point.y);
                search_range = sqrt(pow(rand_point.x - start_tree_[0]->x_, 2) + pow(rand_point.y - start_tree_[0]->y_, 2)) + 
                               sqrt(pow(rand_point.x - goal_tree_[0]->x_, 2) + pow(rand_point.y - goal_tree_[0]->y_, 2));
            }
            trimTheTree(rand_point);
        }
        return ans;
    }

    geometry_msgs::Pose2D InformedRrt::generateRandPoint() {
        geometry_msgs::Pose2D ans;
        int num_point_in_map = ref_map_.info.width * ref_map_.info.height;
        while(ros::ok()) {
            int index = rand() % num_point_in_map;
            if(ori_map_.data[index] > -1) {
                mapToWorld(index % ref_map_.info.width, index / ref_map_.info.width, ans.x, ans.y);
                
                //displayPoint(ans.x, ans.y);
                return ans;
            }
        }
        return ans;
    }

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
        if(ref_map_.data[index] < pose_valid_threshold_) {
            return 0;
        }
        else {
            return 1;
        }
    }

    void InformedRrt::extendTheTree(geometry_msgs::Pose2D point) {
        bool extend_start_tree = generateValidTreeNode(point, start_tree_);
        bool extend_goal_tree = generateValidTreeNode(point, goal_tree_);
        if(extend_start_tree && !extend_goal_tree) {
            if(findPathCheck(start_tree_, goal_tree_) == true) {
                getTheInitPath();
            }
        }
        else if(!extend_start_tree && extend_goal_tree) {
            if(findPathCheck(goal_tree_, start_tree_) == true) {
                getTheInitPath();
            }
        }
        else if(extend_start_tree && extend_goal_tree) {
            if(findPathCheck(start_tree_, goal_tree_) == true) {
                getTheInitPath();
            }
            if(findPathCheck(goal_tree_, start_tree_) == true) {
                getTheInitPath();
            }
        }
    }

    void InformedRrt::trimTheTree(geometry_msgs::Pose2D point) {
        bool extend_start_tree = generateValidTreeNode(point, start_tree_);
        bool extend_goal_tree = generateValidTreeNode2(point, goal_tree_);
        if(extend_start_tree && !extend_goal_tree) {
            // for(int i = 0; i < start_tree_.size(); i++) {
            //     if(connectTwoNode(start_tree_[start_tree_.size() - 1], start_tree_[i], true)) {
            //         getTheInitPath();
            //     }
            // }
        }
        else if(!extend_start_tree && extend_goal_tree) {
        }
        else if(extend_start_tree && extend_goal_tree) {
        }
    }

    bool InformedRrt::generateValidTreeNode(geometry_msgs::Pose2D point, vector<Node*>& tree) {
        bool ans = false;
        Node* new_node;
        double min_d = 99999;
        int index = 0;
        for(int i = 0; i < tree.size(); i++) {
            double dist = pow(point.x - tree[i]->x_, 2) + pow(point.y - tree[i]->y_, 2);
            if(min_d > dist) {
                index = i;
                min_d = dist;
            }
        }
        double x, y, theta;
        x = tree[index]->x_ + step_ / sqrt(pow(tree[index]->x_ - point.x, 2) + pow(tree[index]->y_ - point.y, 2)) * (point.x - tree[index]->x_);
        y = tree[index]->y_ + step_ / sqrt(pow(tree[index]->x_ - point.x, 2) + pow(tree[index]->y_ - point.y, 2)) * (point.y - tree[index]->y_);
        if(obstacleCheck(x, y) == 1) {
            return false;
        }
        theta = atan((point.y - tree[index]->y_) / (point.x - tree[index]->x_));
        if((point.y - tree[index]->y_ > 0) && (point.x - tree[index]->x_ < 0)) {
            theta = theta + PI;
        }
        else if((point.y - tree[index]->y_ < 0) && (point.x - tree[index]->x_ < 0)) {
            theta = theta - PI;
        }
        
        new_node = new Node(x, y, theta);

        for(int i = 0; i < tree.size(); i++) {
            if(pow(x - tree[i]->x_, 2) + pow(y - tree[i]->y_, 2) < pow(trim_scope_, 2)) {
                if(ans == false) {
                    ans = connectTwoNode(tree[i], new_node, true);
                }
                else {
                    connectTwoNode(tree[i], new_node, true);
                }
            }
            //displayTree();
        }
        if(ans == true) {
            tree.push_back(new_node);
            //cout << "extend the tree." << endl;
        }
        else {
            delete new_node;
        }
        return ans;
    }

    bool InformedRrt::generateValidTreeNode2(geometry_msgs::Pose2D point, vector<Node*>& tree) {
        bool ans = false;
        Node* new_node;
        double min_d = 99999;
        int index = 0;
        for(int i = 0; i < tree.size(); i++) {
            double dist = pow(point.x - tree[i]->x_, 2) + pow(point.y - tree[i]->y_, 2);
            if(min_d > dist) {
                index = i;
                min_d = dist;
            }
        }
        double x, y, theta;
        x = tree[index]->x_ + step_ / sqrt(pow(tree[index]->x_ - point.x, 2) + pow(tree[index]->y_ - point.y, 2)) * (point.x - tree[index]->x_);
        y = tree[index]->y_ + step_ / sqrt(pow(tree[index]->x_ - point.x, 2) + pow(tree[index]->y_ - point.y, 2)) * (point.y - tree[index]->y_);
        if(obstacleCheck(x, y) == 1) {
            return false;
        }
        theta = atan((point.y - tree[index]->y_) / (point.x - tree[index]->x_));
        if((point.y - tree[index]->y_ > 0) && (point.x - tree[index]->x_ < 0)) {
            theta = theta + PI;
        }
        else if((point.y - tree[index]->y_ < 0) && (point.x - tree[index]->x_ < 0)) {
            theta = theta - PI;
        }
        
        x = -0.6;
        y = 0.6;
        theta = -PI;
        new_node = new Node(x, y, theta);
        getchar();

        for(int i = 0; i < tree.size(); i++) {
            if(pow(x - tree[i]->x_, 2) + pow(y - tree[i]->y_, 2) < pow(trim_scope_, 2)) {
                if(ans == false) {
                    ans = connectTwoNode(tree[i], new_node, true);
                }
                else {
                    connectTwoNode(tree[i], new_node, true);
                }
            }
            //displayTree();
        }
        if(ans == true) {
            tree.push_back(new_node);
            //cout << "extend the tree." << endl;
        }
        else {
            delete new_node;
        }
        return ans;
    }

    bool InformedRrt::connectTwoNode(Node* node1, Node* node2, bool connect_flag) {
        bool ans = false;
        double t_f = sqrt(pow(node1->x_ - node2->x_, 2) + pow(node1->y_ - node2->y_, 2)) / ave_v_;
        Matrix<double, 6, 6> A;
        Matrix<double, 6, 1> param_x;
        Matrix<double, 6, 1> param_y;
        Matrix<double, 6, 1> b_x;
        Matrix<double, 6, 1> b_y;
        A(0, 0) = 1; A(0, 1) = 0;   A(0, 2) = 0;           A(0, 3) = 0;               A(0, 4) = 0;                A(0, 5) = 0;
        A(1, 0) = 1; A(1, 1) = t_f; A(1, 2) = pow(t_f, 2); A(1, 3) = pow(t_f, 3);     A(1, 4) = pow(t_f, 4);      A(1, 5) = pow(t_f, 5);
        A(2, 0) = 0; A(2, 1) = 1;   A(2, 2) = 0;           A(2, 3) = 0;               A(2, 4) = 0;                A(2, 5) = 0;
        A(3, 0) = 0; A(3, 1) = 1;   A(3, 2) = 2 * t_f;     A(3, 3) = 3 * pow(t_f, 2); A(3, 4) = 4 * pow(t_f, 3);  A(3, 5) = 5 * pow(t_f, 4);
        A(4, 0) = 0; A(4, 1) = 0;   A(4, 2) = 2;           A(4, 3) = 0;               A(4, 4) = 0;                A(4, 5) = 0;
        A(5, 0) = 0; A(5, 1) = 0;   A(5, 2) = 2;           A(5, 3) = 6 * t_f;         A(5, 4) = 12 * pow(t_f, 2); A(5, 5) = 20 * pow(t_f, 3);
        b_x(0, 0) = node1->x_; b_y(0, 0) = node1->y_;
        b_x(1, 0) = node2->x_; b_y(1, 0) = node2->y_;
        double x_v = ave_v_ / sqrt(1 + pow(tan(node1->theta_), 2));
        double y_v = x_v * fabs(tan(node1->theta_));
        x_v = fabs(node1->theta_) < PI / 2 ? x_v : -x_v;
        y_v = node1->theta_ > 0 ? y_v : -y_v;
        b_x(2, 0) = x_v;    b_y(2, 0) = y_v;
        x_v = ave_v_ / sqrt(1 + pow(tan(node2->theta_), 2));
        y_v = x_v * fabs(tan(node2->theta_));
        x_v = fabs(node2->theta_) < PI / 2 ? x_v : -x_v;
        y_v = node2->theta_ > 0 ? y_v : -y_v;
        b_x(3, 0) = x_v;    b_y(3, 0) = y_v;
        b_x(4, 0) = 0;         b_y(4, 0) = 0;
        b_x(5, 0) = 0;         b_y(5, 0) = 0;
        param_x = A.inverse() * b_x;
        param_y = A.inverse() * b_y;

        double dist = 0;
        double last_x, last_y;
        for(double t = 0.0; t < t_f; t+=0.01) {
            double x, y;
            x = param_x(0, 0) + param_x(1, 0) * t + param_x(2, 0) * pow(t, 2) 
              + param_x(3, 0) * pow(t, 3) + param_x(4, 0) * pow(t, 4) + param_x(5, 0) * pow(t, 5);
            y = param_y(0, 0) + param_y(1, 0) * t + param_y(2, 0) * pow(t, 2) 
              + param_y(3, 0) * pow(t, 3) + param_y(4, 0) * pow(t, 4) + param_y(5, 0) * pow(t, 5);
            if(obstacleCheck(x, y) == 1) {
                return false;
            }
            if(t > 0.0) {
                dist += sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
            }

            last_x = x;
            last_y = y;
        }
        dist_of_two_node_ = dist;
        if(connect_flag == true) {
            if(node2->dist_to_root_ > node1->dist_to_root_ + dist) {
                node2->dist_to_root_ = node1->dist_to_root_ + dist;
                node2->father_node_ = node1;
            }
        }

        //displayTree();

        return true;
    }

    bool InformedRrt::findPathCheck(vector<Node*>& tree1, vector<Node*>& tree2) {
        bool ans = false;
        for(int i = 0; i < tree2.size(); i++) {
            if(pow(tree1[tree1.size() - 1]->x_ - tree2[i]->x_, 2) + pow(tree1[tree1.size() - 1]->y_ - tree2[i]->y_, 2) < trim_scope_ * trim_scope_) {
                if(connectTwoNode(tree1[tree1.size() - 1], tree2[i], false) == true) {
                    if(tree1[tree1.size() - 1]->dist_of_path_ > tree1[tree1.size() - 1]->dist_to_root_ + dist_of_two_node_ + tree2[i]->dist_to_root_) {
                        tree1[tree1.size() - 1]->dist_of_path_ = tree1[tree1.size() - 1]->dist_to_root_ + dist_of_two_node_ + tree2[i]->dist_to_root_;
                        tree1[tree1.size() - 1]->joint_node_ = tree2[i];
                        ans = true;
                        joint_node_ = tree1[tree1.size() - 1];
                    }
                }
            }
        }
        return ans;
    }

    void InformedRrt::getTheInitPath() {
        Node* interim_node = joint_node_;
        vector<Node*> part_of_path0;
        vector<Node*> part_of_path1;

        part_of_path0.push_back(joint_node_);
        while(interim_node->father_node_ != NULL) {
            part_of_path0.push_back(interim_node->father_node_);
            interim_node = interim_node->father_node_;
        }

        part_of_path1.push_back(joint_node_->joint_node_);
        while(part_of_path1[part_of_path1.size() - 1]->father_node_ != NULL) {
            part_of_path1.push_back(part_of_path1[part_of_path1.size() - 1]->father_node_);
        }

        path_.poses.clear();
        if(fabs(part_of_path0[part_of_path0.size() - 1]->x_ - start_tree_[0]->x_) + fabs(part_of_path0[part_of_path0.size() - 1]->y_ - start_tree_[0]->y_) <
           fabs(part_of_path0[part_of_path0.size() - 1]->x_ - goal_tree_[0]->x_) + fabs(part_of_path0[part_of_path0.size() - 1]->y_ - goal_tree_[0]->y_)) {
            reverse(part_of_path0.begin(), part_of_path0.end());
            for(int i = 0; i < part_of_path0.size(); i++) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = part_of_path0[i]->x_;
                pose.pose.position.y = part_of_path0[i]->y_;
                pose.pose.position.z = 0.1;
                pose.pose.orientation.w = cos(part_of_path0[i]->theta_ / 2);
                pose.pose.orientation.z = sin(part_of_path0[i]->theta_ / 2);
                path_.poses.push_back(pose);
            }
            for(int i = 0; i < part_of_path1.size(); i++) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = part_of_path1[i]->x_;
                pose.pose.position.y = part_of_path1[i]->y_;
                pose.pose.position.z = 0.1;
                pose.pose.orientation.w = cos(part_of_path1[i]->theta_ / 2);
                pose.pose.orientation.z = sin(part_of_path1[i]->theta_ / 2);
                path_.poses.push_back(pose);
            }
        }
        else {
            reverse(part_of_path1.begin(), part_of_path1.end());
            for(int i = 0; i < part_of_path1.size(); i++) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = part_of_path1[i]->x_;
                pose.pose.position.y = part_of_path1[i]->y_;
                pose.pose.position.z = 0.1;
                pose.pose.orientation.w = cos(part_of_path1[i]->theta_ / 2);
                pose.pose.orientation.z = sin(part_of_path1[i]->theta_ / 2);
                path_.poses.push_back(pose);
            }
            for(int i = 0; i < part_of_path0.size(); i++) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = part_of_path0[i]->x_;
                pose.pose.position.y = part_of_path0[i]->y_;
                pose.pose.position.z = 0.1;
                pose.pose.orientation.w = cos(part_of_path0[i]->theta_ / 2);
                pose.pose.orientation.z = sin(part_of_path0[i]->theta_ / 2);
                path_.poses.push_back(pose);
            }
        }
        pub_path_.publish(path_);
        cout << "display the path." << endl;

        /////////////////////////////
        double dist = 0;
        for(int i = 1; i < path_.poses.size(); i++) {
            double dx = path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x;
            double dy = path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y;
            dist += sqrt(pow(dx, 2) + pow(dy, 2));
        }
        cout << "the distance of path may is: " << dist << ", " << joint_node_->dist_of_path_ << endl;
        /////////////////////////////
    }

    double InformedRrt::getPathLength() {
        double dist = 0;
        for(int i = 1; i < path_.poses.size(); i++) {
            double dx = path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x;
            double dy = path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y;
            dist += sqrt(pow(dx, 2) + pow(dy, 2));
        }
        return dist;
    }

    void InformedRrt::displayTree() {
        //cout << "display the rrt tree." << endl;
        sensor_msgs::PointCloud tree;
        tree.header.frame_id = "map";
        sensor_msgs::PointCloud node;
        node.header.frame_id = "map";
        for(int i = 1; i < start_tree_.size(); i++) {
            double t_f = sqrt(pow(start_tree_[i]->x_ - start_tree_[i]->father_node_->x_, 2) + pow(start_tree_[i]->y_ - start_tree_[i]->father_node_->y_, 2)) / ave_v_;
            Matrix<double, 6, 6> A;
            Matrix<double, 6, 1> param_x;
            Matrix<double, 6, 1> param_y;
            Matrix<double, 6, 1> b_x;
            Matrix<double, 6, 1> b_y;
            A(0, 0) = 1; A(0, 1) = 0;   A(0, 2) = 0;           A(0, 3) = 0;               A(0, 4) = 0;                A(0, 5) = 0;
            A(1, 0) = 1; A(1, 1) = t_f; A(1, 2) = pow(t_f, 2); A(1, 3) = pow(t_f, 3);     A(1, 4) = pow(t_f, 4);      A(1, 5) = pow(t_f, 5);
            A(2, 0) = 0; A(2, 1) = 1;   A(2, 2) = 0;           A(2, 3) = 0;               A(2, 4) = 0;                A(2, 5) = 0;
            A(3, 0) = 0; A(3, 1) = 1;   A(3, 2) = 2 * t_f;     A(3, 3) = 3 * pow(t_f, 2); A(3, 4) = 4 * pow(t_f, 3);  A(3, 5) = 5 * pow(t_f, 4);
            A(4, 0) = 0; A(4, 1) = 0;   A(4, 2) = 2;           A(4, 3) = 0;               A(4, 4) = 0;                A(4, 5) = 0;
            A(5, 0) = 0; A(5, 1) = 0;   A(5, 2) = 2;           A(5, 3) = 6 * t_f;         A(5, 4) = 12 * pow(t_f, 2); A(5, 5) = 20 * pow(t_f, 3);
            b_x(0, 0) = start_tree_[i]->father_node_->x_; b_y(0, 0) = start_tree_[i]->father_node_->y_;
            b_x(1, 0) = start_tree_[i]->x_;     b_y(1, 0) = start_tree_[i]->y_;
            double x_v = ave_v_ / sqrt(1 + pow(tan(start_tree_[i]->father_node_->theta_), 2));
            double y_v = x_v * fabs(tan(start_tree_[i]->father_node_->theta_));
            x_v = fabs(start_tree_[i]->father_node_->theta_) < PI / 2 ? x_v : -x_v;
            y_v = start_tree_[i]->father_node_->theta_ > 0 ? y_v : -y_v;
            b_x(2, 0) = x_v;    b_y(2, 0) = y_v;
            x_v = ave_v_ / sqrt(1 + pow(tan(start_tree_[i]->theta_), 2));
            y_v = x_v * fabs(tan(start_tree_[i]->theta_));
            x_v = fabs(start_tree_[i]->theta_) < PI / 2 ? x_v : -x_v;
            y_v = start_tree_[i]->theta_ > 0 ? y_v : -y_v;
            b_x(3, 0) = x_v;       b_y(3, 0) = y_v;
            b_x(4, 0) = 0;         b_y(4, 0) = 0;
            b_x(5, 0) = 0;         b_y(5, 0) = 0;
            param_x = A.inverse() * b_x;
            param_y = A.inverse() * b_y;

            geometry_msgs::Point32 one_point;
            for(double t = 0.01; t < t_f; t+=0.01) {
                double x, y;
                one_point.x = param_x(0, 0) + param_x(1, 0) * t + param_x(2, 0) * pow(t, 2) 
                            + param_x(3, 0) * pow(t, 3) + param_x(4, 0) * pow(t, 4) + param_x(5, 0) * pow(t, 5);
                one_point.y = param_y(0, 0) + param_y(1, 0) * t + param_y(2, 0) * pow(t, 2) 
                            + param_y(3, 0) * pow(t, 3) + param_y(4, 0) * pow(t, 4) + param_y(5, 0) * pow(t, 5);
                one_point.z = 0.1;
                tree.points.push_back(one_point);
            }
            one_point.x = start_tree_[i]->x_;
            one_point.y = start_tree_[i]->y_;
            one_point.z = 0.1;
            node.points.push_back(one_point);
        }
        for(int i = 1; i < goal_tree_.size(); i++) {
            double t_f = sqrt(pow(goal_tree_[i]->x_ - goal_tree_[i]->father_node_->x_, 2) + pow(goal_tree_[i]->y_ - goal_tree_[i]->father_node_->y_, 2)) / ave_v_;
            Matrix<double, 6, 6> A;
            Matrix<double, 6, 1> param_x;
            Matrix<double, 6, 1> param_y;
            Matrix<double, 6, 1> b_x;
            Matrix<double, 6, 1> b_y;
            A(0, 0) = 1; A(0, 1) = 0;   A(0, 2) = 0;           A(0, 3) = 0;               A(0, 4) = 0;                A(0, 5) = 0;
            A(1, 0) = 1; A(1, 1) = t_f; A(1, 2) = pow(t_f, 2); A(1, 3) = pow(t_f, 3);     A(1, 4) = pow(t_f, 4);      A(1, 5) = pow(t_f, 5);
            A(2, 0) = 0; A(2, 1) = 1;   A(2, 2) = 0;           A(2, 3) = 0;               A(2, 4) = 0;                A(2, 5) = 0;
            A(3, 0) = 0; A(3, 1) = 1;   A(3, 2) = 2 * t_f;     A(3, 3) = 3 * pow(t_f, 2); A(3, 4) = 4 * pow(t_f, 3);  A(3, 5) = 5 * pow(t_f, 4);
            A(4, 0) = 0; A(4, 1) = 0;   A(4, 2) = 2;           A(4, 3) = 0;               A(4, 4) = 0;                A(4, 5) = 0;
            A(5, 0) = 0; A(5, 1) = 0;   A(5, 2) = 2;           A(5, 3) = 6 * t_f;         A(5, 4) = 12 * pow(t_f, 2); A(5, 5) = 20 * pow(t_f, 3);
            b_x(0, 0) = goal_tree_[i]->father_node_->x_; b_y(0, 0) = goal_tree_[i]->father_node_->y_;
            b_x(1, 0) = goal_tree_[i]->x_;     b_y(1, 0) = goal_tree_[i]->y_;
            double x_v = ave_v_ / sqrt(1 + pow(tan(goal_tree_[i]->father_node_->theta_), 2));
            double y_v = x_v * fabs(tan(goal_tree_[i]->father_node_->theta_));
            x_v = fabs(goal_tree_[i]->father_node_->theta_) < PI / 2 ? x_v : -x_v;
            y_v = goal_tree_[i]->father_node_->theta_ > 0 ? y_v : -y_v;
            b_x(2, 0) = x_v;    b_y(2, 0) = y_v;
            x_v = ave_v_ / sqrt(1 + pow(tan(goal_tree_[i]->theta_), 2));
            y_v = x_v * fabs(tan(goal_tree_[i]->theta_));
            x_v = fabs(goal_tree_[i]->theta_) < PI / 2 ? x_v : -x_v;
            y_v = goal_tree_[i]->theta_ > 0 ? y_v : -y_v;
            b_x(3, 0) = x_v;       b_y(3, 0) = y_v;
            b_x(4, 0) = 0;         b_y(4, 0) = 0;
            b_x(5, 0) = 0;         b_y(5, 0) = 0;
            param_x = A.inverse() * b_x;
            param_y = A.inverse() * b_y;

            geometry_msgs::Point32 one_point;
            for(double t = 0.01; t < t_f; t+=0.01) {
                double x, y;
                one_point.x = param_x(0, 0) + param_x(1, 0) * t + param_x(2, 0) * pow(t, 2) 
                            + param_x(3, 0) * pow(t, 3) + param_x(4, 0) * pow(t, 4) + param_x(5, 0) * pow(t, 5);
                one_point.y = param_y(0, 0) + param_y(1, 0) * t + param_y(2, 0) * pow(t, 2) 
                            + param_y(3, 0) * pow(t, 3) + param_y(4, 0) * pow(t, 4) + param_y(5, 0) * pow(t, 5);
                one_point.z = 0.1;
                tree.points.push_back(one_point);
            }
            one_point.x = goal_tree_[i]->x_;
            one_point.y = goal_tree_[i]->y_;
            one_point.z = 0.1;
            node.points.push_back(one_point);
        }
        pub_tree_.publish(tree);
        pub_node_.publish(node);
        // cout << "the scale of trees: " <<  start_tree_.size() << ", " << goal_tree_.size() << endl;
        // for(int i = 0; i < start_tree_.size(); i++) {
        //     cout << start_tree_[i]->x_ << ", " << start_tree_[i]->y_ << ", " << start_tree_[i]->dist_to_root_ << endl;
        // }
        // cout << "other tree." << endl;
        // for(int i = 0; i < goal_tree_.size(); i++) {
        //     cout << goal_tree_[i]->x_ << ", " << goal_tree_[i]->y_ << ", " << goal_tree_[i]->dist_to_root_ << endl;
        // }

        //getchar();
    }

    void InformedRrt::displayPoint(double x, double y) {
        cout << "display a point: " << x << ", " << y << endl;
        sensor_msgs::PointCloud point;
        point.header.frame_id = "map";
        point.points.resize(1);
        point.points[0].x = x;
        point.points[0].y = y;
        point.points[0].z = 0.03;
        pub_point_.publish(point);
        getchar();
    }
};