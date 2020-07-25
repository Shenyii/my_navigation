#ifndef NODE_H
#define NODE_H

#include <iostream>

using namespace std;

class Node{
public:
    double x_;
    double y_;
    double theta_;
    Node* father_node_;
    Node* joint_node_;
    double dist_to_root_;
    double dist_of_path_;
    bool goal_flag_;

    Node();
    ~Node();
    Node(double x, double y, double theta);
};

#endif
