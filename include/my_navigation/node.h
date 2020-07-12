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
    double distance_;

    Node();
    ~Node();
    Node(double x, double y, double theta);
};

#endif
