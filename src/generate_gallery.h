#ifndef GENERATE_GALLERY_H
#define GENERATE_GALLERY_H

#include <iostream>
#include <vector>
#include <list>

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;

class Box {
public:
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    Box() {
        x_min_ = 0;
        x_max_ = 0;
        y_min_ = 0;
        y_max_ = 0;
    };
    Box(double x0, double x1, double y0, double y1) {
        x_min_ = x0;
        x_max_ = x1;
        y_min_ = y0;
        y_max_ = y1;
    };
};

class GenerateGallery {
public:
    GenerateGallery();
    ~GenerateGallery();

private:
    double obstacle_threshold_;
    double map_resolution_;
    vector<Box> gallerys_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool nodeObstacleCheck(double x, double y);
    bool generateGallery();
    bool extendBox(double x0, double y0, double x1, double y1);
    bool pathNodeInBox(double x, double y, Box box);

    ros::NodeHandle nh_;
    nav_msgs::Path ori_path_;
    ros::Subscriber sub_path_;
    nav_msgs::OccupancyGrid ori_map_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_gallerys_;

    void subPath(nav_msgs::Path path);
    void subMap(nav_msgs::OccupancyGrid map);
    void displayGallery();
};

#endif