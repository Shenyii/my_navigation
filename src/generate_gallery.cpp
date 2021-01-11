#include "generate_gallery.h"

GenerateGallery::GenerateGallery() {
    sub_path_ = nh_.subscribe("/own_path", 2, &GenerateGallery::subPath, this);
    sub_map_ = nh_.subscribe("/map", 2, &GenerateGallery::subMap, this);
    pub_gallerys_ = nh_.advertise<visualization_msgs::MarkerArray>("/agvs", 2);
    pub_gallerys2_ = nh_.advertise<std_msgs::Float32MultiArray>("/gallerys", 2);
    obstacle_threshold_ = 65;
}

GenerateGallery::~GenerateGallery() {}

void GenerateGallery::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

void GenerateGallery::subPath(nav_msgs::Path path) {
    ori_path_ = path;

    double det_time = ros::Time::now().toSec();
    generateGallery();
    cout << "cost time of generate gallerys: " << ros::Time::now().toSec() - det_time << endl;
}

void GenerateGallery::subMap(nav_msgs::OccupancyGrid map) {
    ori_map_ = map;
    map_resolution_ = ori_map_.info.resolution;
}

bool GenerateGallery::nodeObstacleCheck(double x, double y) {
    int mx;
    int my;
    worldToMap(x, y, mx, my);
    int test = ori_map_.data[mx + my * ori_map_.info.width];
    if(ori_map_.data[mx + my * ori_map_.info.width] < obstacle_threshold_ && ori_map_.data[mx + my * ori_map_.info.width] != -1) {
        return false;
    }
    else {
        return true;
    }
}

bool GenerateGallery::generateGallery() {
    bool ans;
    gallerys_.clear();
    vector<geometry_msgs::Point> path;
    path.resize(ori_path_.poses.size());
    for(int i = 0; i < ori_path_.poses.size(); i++) {
        path[i].x = ori_path_.poses[i].pose.position.x;
        path[i].y = ori_path_.poses[i].pose.position.y;
        path[i].z = ori_path_.poses[i].pose.position.z;
    }
    Box box(path[0].x, path[0].x, path[0].y, path[0].y);
    bool ext_x_min = 1;
    bool ext_x_max = 1;
    bool ext_y_min = 1;
    bool ext_y_max = 1;
    int n = 0;
    while(path.size() != 0 && ros::ok()) {
        if(path[0].x >= box.x_min_ - 0.5* map_resolution_ && 
           path[0].x <= box.x_max_ + 0.5* map_resolution_ && 
           path[0].y >= box.y_min_ - 0.5* map_resolution_ && 
           path[0].y <= box.y_max_ + 0.5* map_resolution_) {
            path.erase(path.begin(), path.begin() + 1);
            continue;
        }
        if(ext_x_min == 0 && ext_x_max == 0 && ext_y_min == 0 && ext_y_max == 0) {
            gallerys_.push_back(box);
            box.x_min_ = path[0].x;
            box.x_max_ = path[0].x;
            box.y_min_ = path[0].y;
            box.y_max_ = path[0].y;
            ext_x_min = 1;
            ext_x_max = 1;
            ext_y_min = 1;
            ext_y_max = 1;
        }
        if(path[0].x < box.x_min_) {
            //cout << "left" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
        }
        else if(path[0].y < box.y_min_) {
            //cout << "down" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
        }
        else if(path[0].x > box.x_max_) {
            //cout << "right" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
            //cout << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
        }
        else if(path[0].y > box.y_max_) {
            //cout << "up" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
        }
        
        // cout << path.size() << ", " << path[0].x << ", " << path[0].y << endl;
        // cout << box.x_min_ << ", " << box.x_max_ << ", " << box.y_min_ << ", " << box.y_max_ << endl;
        // cout << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
    }
    while(ext_x_min == 1 || ext_x_max == 1 || ext_y_min == 1 || ext_y_max == 1 && ros::ok()) {
        if(ext_x_min == 1) {
            if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                box.x_min_ -= map_resolution_;
            }
            else {
                ext_x_min = 0;
            }
        }
        if(ext_y_min == 1) {
            if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                box.y_min_ -= map_resolution_;
            }
            else {
                ext_y_min = 0;
            }
        }
        if(ext_x_max == 1) {
            if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                box.x_max_ += map_resolution_;
            }
            else {
                ext_x_max = 0;
            }
        }
        if(ext_y_max == 1) {
            if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                box.y_max_ += map_resolution_;
            }
            else {
                ext_y_max = 0;
            }
        }
    }
    gallerys_.push_back(box);

    pubGallerys();

    displayGallery();

    return ans;
}

bool GenerateGallery::extendBox(double x0, double y0, double x1, double y1) {
    bool ans = true;
    double d = hypot(x0 - x1, y0 - y1);
    if(d == 0) {
        if(nodeObstacleCheck(x0, y0) == 1) {
            return false;
        }
        else {
            return true;
        }
    }
    for(int i = 0; i <= d / map_resolution_; i++) {
        double x = x0 + double(i) * (x1 - x0) * map_resolution_ / d;
        double y = y0 + double(i) * (y1 - y0) * map_resolution_ / d;
        if(nodeObstacleCheck(x, y) == 1) {
            ans = false;
            break;
        }
    }

    return ans;
}

bool GenerateGallery::pathNodeInBox(double x, double y, Box box) {
    bool ans = false;
    if(x >= box.x_min_ && x <= box.x_max_ && y >= box.y_min_ && y <= box.y_max_) {}

    return ans;
}

bool GenerateGallery::pointInGallery(double x, double y, Box box, double bias) {
    bool ans = true;
    if(x >= box.x_min_ - bias && x <= box.x_max_ + bias && y >= box.y_min_ - bias && y <= box.y_max_ + bias) {
        return true;
    }
    else {
        return false;
    }

    return ans;
}

bool GenerateGallery::twoGallerysIntersect(Box box0, Box box1) {
    bool ans = true;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    x_min = box0.x_min_ > box1.x_min_ ? box0.x_min_ : box1.x_min_;
    x_max = box0.x_max_ < box1.x_max_ ? box0.x_max_ : box1.x_max_;
    y_min = box0.y_min_ > box1.y_min_ ? box0.y_min_ : box1.y_min_;
    y_max = box0.y_max_ < box1.y_max_ ? box0.y_max_ : box1.y_max_;
    if((x_max - x_min >= 0) && (y_max - y_min >= 0)) {
        return true;
    }
    else {
        return false;
    }

    return ans;
}

void GenerateGallery::pubGallerys() {
    std_msgs::Float32MultiArray gallerys;
    for(int i = 0; i < gallerys_.size() - 1; i++) {
        if(twoGallerysIntersect(gallerys_[i], gallerys_[i + 1]) == 0) {
            cout << "generate error gallerys." << endl;
            return;
        }
    }
    gallerys.layout.dim.resize(gallerys_.size());
    for(int i = 0; i < gallerys_.size(); i++) {
        gallerys.data.push_back(gallerys_[i].x_min_);
        gallerys.data.push_back(gallerys_[i].x_max_);
        gallerys.data.push_back(gallerys_[i].y_min_);
        gallerys.data.push_back(gallerys_[i].y_max_);
    }
    pub_gallerys2_.publish(gallerys);
}

void GenerateGallery::displayGallery() {
    static int last_gallerys_num = 0;
    cout << "display the gallerys. " << gallerys_.size() << endl;
    visualization_msgs::MarkerArray markers;
    for(int i = 0; i < gallerys_.size(); i++) {
        visualization_msgs::Marker module;
        module.header.frame_id = "map";
        module.header.stamp = ros::Time::now();
        module.ns = "";
        module.color.r = 0.5f;
        module.color.g = 0.8f;
        module.color.b = 0.2f;
        module.color.a = 1.0;
        module.lifetime = ros::Duration();
        module.frame_locked = true;
        module.type = visualization_msgs::Marker::CUBE;
        module.action = visualization_msgs::Marker::ADD;
        module.id = i;
        module.pose.position.x = 0.5 * (gallerys_[i].x_min_ + gallerys_[i].x_max_);
        module.pose.position.y = 0.5 * (gallerys_[i].y_min_ + gallerys_[i].y_max_);
        module.pose.position.z = -0.1 -0.05 * i;
        module.scale.x = gallerys_[i].x_max_ - gallerys_[i].x_min_;
        module.scale.y = gallerys_[i].y_max_ - gallerys_[i].y_min_;
        module.scale.z = 0.05;
        markers.markers.push_back(module);

        cout << gallerys_[i].x_min_ << ", " << gallerys_[i].x_max_ << ", "
             << gallerys_[i].y_min_ << ", " << gallerys_[i].y_max_ << endl;
    }
    for(int i = 0; i < last_gallerys_num - int(gallerys_.size()); i++) {
        visualization_msgs::Marker module;
        module.header.frame_id = "map";
        module.header.stamp = ros::Time::now();
        module.ns = "";
        module.color.r = 0.5f;
        module.color.g = 0.8f;
        module.color.b = 0.2f;
        module.color.a = 0;
        module.lifetime = ros::Duration();
        module.frame_locked = true;
        module.type = visualization_msgs::Marker::CUBE;
        module.action = visualization_msgs::Marker::ADD;
        module.id = i + gallerys_.size();
        module.pose.position.x = 0;
        module.pose.position.y = 0;
        module.pose.position.z = 0;
        module.scale.x = 0.1;
        module.scale.y = 0.1;
        module.scale.z = 0.1;
        markers.markers.push_back(module);
    }

    pub_gallerys_.publish(markers);
    last_gallerys_num = gallerys_.size();
}

int main(int argc, char** argv) {
    cout << "begin generate gallery." << endl;
    ros::init(argc, argv, "generate_gallery");
    GenerateGallery test;
    ros::spin();
    return 0;
}