# -*- coding: utf-8 -*-

import rospy
import casadi as ca
import casadi.tools as ca_tools
import numpy as np
import time
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class Box:
    x_min_ = 0
    x_max_ = 0
    y_min_ = 0
    y_max_ = 0

    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min_ = x_min
        self.x_max_ = x_max
        self.y_min_ = y_min
        self.y_max_ = y_max

class OptPathInGenerate:
    curve_num_ = 0    #曲线条数
    max_curvative_ = 2    #最大曲率半径
    gallerys_ = []
    #通过查表获取高斯点
    gauss_points_ = [-0.9782286581460570, -0.8870625997680950, -0.7301520055740490, -0.5190961292068110, -0.2695431559523440, 0.0000000000000000, 0.2695431559523440, 0.5190961292068110, 0.7301520055740490, 0.8870625997680950, 0.9782286581460570]
    gauss_weight_ = [0.0556685671161736, 0.1255803694649040, 0.1862902109277340, 0.2331937645919900, 0.2628045445102460, 0.2729250867779000, 0.2628045445102460, 0.2331937645919900, 0.1862902109277340, 0.1255803694649040, 0.0556685671161736]
    
    Q_ = np.array([[1.0, 0.0, 0.0, 0.0],[0.0, 1.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])    #定义权重矩阵
    X_ = ca.SX.sym('X_', 4, 1)    #定义状态变量
    Y_ = ca.SX.sym('Y_', 1, 1)    #定义函数值
    x_ = ca.SX.sym('x_', 1, 1)    #定义变量
    Y_ = 0
    #定义使用高斯——勒让德求解积分的函数
    for i in range(len(gauss_points_)):
        x_ = 0.5 * X_[3, 0] * gauss_points_[i] + 0.5 * X_[3, 0]
        Y_ = Y_ + gauss_weight_[i] * ca.cos(0.5*X_[0, 0]*x_*x_+X_[1, 0]*x_+X_[2, 0])
    Y_ = 0.5 * Y_ * X_[3, 0]
    FC = ca.Function('FC', [X_[0, 0], X_[1, 0], X_[2, 0], X_[3, 0]], [Y_], ['param', 'init_cur', 'init_theta', 'arc_s'], ['output'])
    Y_ = 0
    for i in range(len(gauss_points_)):
        x_ = 0.5 * X_[3, 0] * gauss_points_[i] + 0.5 * X_[3, 0]
        Y_ = Y_ + gauss_weight_[i] * ca.sin(0.5*X_[0, 0]*x_*x_+X_[1, 0]*x_+X_[2, 0])
    Y_ = 0.5 * Y_ * X_[3, 0]
    FS = ca.Function('FS', [X_[0, 0], X_[1, 0], X_[2, 0], X_[3, 0]], [Y_], ['param', 'init_cur', 'init_theta', 'arc_s'], ['output'])

    def __init__(self):
        print("casadi opt clothoid")
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.callBackStartPose)    #订阅起点消息
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callBackGoalPose)    #订阅终点消息
        rospy.Subscriber('/gallerys', Float32MultiArray, self.callBackGallerys)    #订阅廊道消息
        self.pub_clothoid_ = rospy.Publisher('/pre_trajectory', PointCloud, queue_size = 2)    #发布点云消息
        #初始化起点状态
        self.s_x_ = 0
        self.s_y_ = 0
        self.s_theta_ = 0
        self.s_curvative_ = 0
        #初始化终点状态
        self.g_x_ = 3
        self.g_y_ = 1
        self.g_theta_ = 0.6
        self.g_curvative_ = 0

    def callBackGallerys(self, Float32MultiArray):
        print('get the gallerys')
        self.gallerys_ = []
        for i in range(len(Float32MultiArray.data) / 4):
            self.gallerys_.append(Box(Float32MultiArray.data[4 * i], Float32MultiArray.data[4 * i + 1], Float32MultiArray.data[4 * i + 2], Float32MultiArray.data[4 * i + 3]))
        for i in range(len(self.gallerys_)):
            print(self.gallerys_[i].x_min_, self.gallerys_[i].x_max_, self.gallerys_[i].y_min_, self.gallerys_[i].y_max_)
        gallerys = []
        for i in range(len(self.gallerys_) - 1):
            gallerys.append(self.gallerys_[i])
            x_min = self.gallerys_[i].x_min_ if self.gallerys_[i].x_min_ > self.gallerys_[i + 1].x_min_ else self.gallerys_[i + 1].x_min_
            x_max = self.gallerys_[i].x_max_ if self.gallerys_[i].x_max_ < self.gallerys_[i + 1].x_max_ else self.gallerys_[i + 1].x_max_
            y_min = self.gallerys_[i].y_min_ if self.gallerys_[i].y_min_ > self.gallerys_[i + 1].y_min_ else self.gallerys_[i + 1].y_min_
            y_max = self.gallerys_[i].y_max_ if self.gallerys_[i].y_max_ < self.gallerys_[i + 1].y_max_ else self.gallerys_[i + 1].y_max_
            gallerys.append(Box(x_min, x_max, y_min, y_max))
        gallerys.append(self.gallerys_[-1])
        print('gallerys:')
        for i in range(len(gallerys)):
            print(gallerys[i].x_min_, gallerys[i].x_max_, gallerys[i].y_min_, gallerys[i].y_max_)
        self.solveTheProblem(gallerys)

    #订阅起点的回调函数
    def callBackStartPose(self, PoseWithCovarianceStamped):
        self.s_x_ = PoseWithCovarianceStamped.pose.pose.position.x
        self.s_y_ = PoseWithCovarianceStamped.pose.pose.position.y
        self.s_theta_ = math.acos(2 * PoseWithCovarianceStamped.pose.pose.orientation.w * PoseWithCovarianceStamped.pose.pose.orientation.w - 1)
        if(PoseWithCovarianceStamped.pose.pose.orientation.w * PoseWithCovarianceStamped.pose.pose.orientation.z < 0):
            self.s_theta_ = -self.s_theta_
        self.s_curvative_ = 0
        print(self.s_x_, self.s_y_, self.s_theta_)

    #订阅终点的回调函数
    def callBackGoalPose(self, PoseStamped):
        self.g_x_ = PoseStamped.pose.position.x
        self.g_y_ = PoseStamped.pose.position.y
        self.g_theta_ = math.acos(2 * PoseStamped.pose.orientation.w * PoseStamped.pose.orientation.w - 1)
        if(PoseStamped.pose.orientation.w * PoseStamped.pose.orientation.z < 0):
            self.g_theta_ = -self.g_theta_
        self.g_curvative_ = 0
        print(self.g_x_, self.g_y_, self.g_theta_)

    def solveTheProblem(self, gallerys):
        print('pose:', self.s_x_, self.s_y_, self.s_theta_, self.s_curvative_, self.g_x_, self.g_y_, self.g_theta_, self.g_curvative_)
        print_time = rospy.Time().now().to_sec()
        curve_num = 2 * len(self.gallerys_)
        X = ca.SX.sym('X', 2 * curve_num)    #定义优化求解变量
        E = ca.SX.sym('E', 4)    #定义误差向量
        init_X = ca.SX.sym('init_X', 4)    #定义初始状态变量
        init_X[0] = self.s_x_
        init_X[1] = self.s_y_
        init_X[2] = self.s_theta_
        init_X[3] = self.s_curvative_
        g = []    #声明约束方程组
        for i in range(curve_num):
            init_X[0] += self.FC(X[2 * i], init_X[3], init_X[2], X[2 * i + 1])
            init_X[1] += self.FS(X[2 * i], init_X[3], init_X[2], X[2 * i + 1])
            init_X[2] += 0.5 * X[2 * i] * X[2 * i + 1] * X[2 * i + 1] + init_X[3] * X[2 * i + 1]
            init_X[3] += X[2 * i] * X[2 * i + 1]
            g.append(init_X[0])    #添加约束方程
            g.append(init_X[1])    #添加约束方程
            g.append(init_X[3])    #添加约束方程
        E[0] = init_X[0] - self.g_x_
        E[1] = init_X[1] - self.g_y_
        E[2] = init_X[2] - self.g_theta_
        E[3] = init_X[3] - self.g_curvative_
        obj = ca.mtimes([E.T, self.Q_, E])    #定义目标函数
        nlp_prob = {'f': obj, 'x': X, 'g': ca.vertcat(*g)}    #加载目标函数、求解变量、约束方程组
        #设置求解器
        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
        lbx = []
        ubx = []
        lbg = []
        ubg = []
        for i in range(curve_num):
            lbx.append(-10), ubx.append(10)
            lbx.append(0), ubx.append(5)
        for i in range(len(gallerys)):
            lbg.append(gallerys[i].x_min_), ubg.append(gallerys[i].x_max_)
            lbg.append(gallerys[i].y_min_), ubg.append(gallerys[i].y_max_)
            lbg.append(-2), ubg.append(2)
        lbg.append(gallerys[-1].x_min_), ubg.append(gallerys[-1].x_max_)
        lbg.append(gallerys[-1].y_min_), ubg.append(gallerys[-1].y_max_)
        lbg.append(-2), ubg.append(2)
        res = solver(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)    #执行求解计算
        
        print(res)
        x = []
        for i in range(2 * curve_num):
            x.append(res['x'][i])
        self.displayClothoid(x)

    def displayClothoid(self, param):
        clothoid_points = PointCloud()
        clothoid_points.header.frame_id = 'map'
        x_list = []
        y_list = []
        x = self.s_x_
        y = self.s_y_
        theta = self.s_theta_
        cur = self.s_curvative_
        for i in range(len(param) / 2):
            for j in range(param[2 * i + 1] * 100):
                x_list.append(x + self.FC(param[2 * i], cur, theta, 0.01 * j))
                y_list.append(y + self.FS(param[2 * i], cur, theta, 0.01 * j))
                clothoid_points.points.append(Point32(x_list[-1], y_list[-1], 0))
            x += self.FC(param[2 * i], cur, theta, param[2 * i + 1])
            y += self.FS(param[2 * i], cur, theta, param[2 * i + 1])
            theta += 0.5 * param[2 * i] * param[2 * i + 1] * param[2 * i + 1] + cur * param[2 * i + 1]
            cur += param[2 * i] * param[2 * i + 1]
        self.pub_clothoid_.publish(clothoid_points)
        print('error: ', x - self.g_x_, y - self.g_y_, theta - self.g_theta_, cur - self.g_curvative_)

if __name__ == '__main__':
    print("start generate path in gallerys")
    rospy.init_node("casadi_opt_path", anonymous = True)
    test = OptPathInGenerate()
    while not rospy.is_shutdown():
        rospy.sleep(3)
        #print("main program is running.")