#include "node.h"

Node::Node()
: x_(0), y_(0), theta_(0), distance_(-1), father_node_(NULL) {}

Node::~Node() {}

Node::Node(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}