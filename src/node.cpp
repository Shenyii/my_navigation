#include "node.h"

Node::Node()
: x_(0), y_(0), theta_(0), dist_to_root_(99999), dist_of_path_(99999), father_node_(NULL), joint_node_(NULL) {}

Node::~Node() {
    // delete father_node_;
    // delete joint_node_;
}

Node::Node(double x, double y, double theta)
: dist_to_root_(99999), dist_of_path_(99999), father_node_(NULL), joint_node_(NULL) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}