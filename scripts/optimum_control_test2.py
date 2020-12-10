# -*- coding: utf-8 -*-
import numpy
import casadi as ca
import matplotlib.pyplot as plt
import math
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

fig = plt.figure()
ax1 = fig.add_subplot(111)
#ax1.set_title('Fig')
plt.xlabel('x(m)')
plt.ylabel('y(m)')

g_pub_clothoid = rospy.Publisher('/trajectory', PointCloud, queue_size = 2)

def solveProblem():
    print_time = rospy.Time().now().to_sec()

    N = 51
    U = ca.SX.sym('U', N)
    x0 = ca.SX.sym('x0', 5)    #初始条件
    x0[0] = -1
    x0[1] = 1
    x0[2] = 0
    x0[3] = 0
    x0[4] = 0
    xf = ca.SX.sym('xf', 5)    #终端状态
    xf[0] = x0[0]
    xf[1] = x0[1]
    xf[2] = x0[2]
    xf[3] = x0[3]
    xf[4] = x0[4]
    obj = U[-1]
    g = []
    det_t = ca.SX.sym('det_t', 1)
    det_t = U[-1] / ((N - 1) / 2)
    for i in range((N - 1) / 2):
        xf[4] += det_t * U[2 * i + 1]
        xf[3] += det_t * U[2 * i]
        xf[2] += det_t * xf[4]
        xf[1] += det_t * xf[3] * ca.sin(xf[2])
        xf[0] += det_t * xf[3] * ca.cos(xf[2])
    g.append(xf[0])
    g.append(xf[1])
    g.append(xf[2])
    g.append(xf[3])
    g.append(xf[4])
    nlp_prob = {'f': obj, 'x': U, 'g': ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
    lbx = []
    ubx = []
    lbg = []
    ubg = []
    for i in range((N - 1) / 2):
        lbx.append(-1)
        ubx.append(1)
        lbx.append(-3)
        ubx.append(3)
    lbx.append(0)
    ubx.append(100)
    for i in range(5):
        lbg.append(0)    #终端约束
        ubg.append(0)    #终端约束
    res = solver(lbx = lbx, ubx = ubx, lbg = lbg, ubg = ubg)

    print('cost time of casadi opt problem: ', rospy.Time().now().to_sec() - print_time)
    print(res['f'], res['x'])
    #print(xf)
    UU = res['x']

    points = PointCloud()
    points.header.frame_id = 'map'
    xf[0] = x0[0]
    xf[1] = x0[1]
    xf[2] = x0[2]
    xf[3] = x0[3]
    xf[4] = x0[4]
    points.points.append(Point32(xf[0], xf[1], 0))
    det_t = ca.SX.sym('det_t', 1)
    det_t = UU[-1] / ((N - 1) / 2)
    for i in range((N - 1) / 2):
        xf[4] += det_t * UU[2 * i + 1]
        xf[3] += det_t * UU[2 * i]
        xf[2] += det_t * xf[4]
        xf[1] += det_t * xf[3] * ca.sin(xf[2])
        xf[0] += det_t * xf[3] * ca.cos(xf[2])
        points.points.append(Point32(xf[0], xf[1], 0))
    g_pub_clothoid.publish(points)
    # #while not rospy.is_shutdown():
    for i in range(100000):
        g_pub_clothoid.publish(points)
        #rospy.sleep(0.1)
    print('end pub')
    

if __name__ == '__main__':
    print('opt control test.')
    rospy.init_node("opt_control_test", anonymous = True)
    solveProblem()
    rospy.spin()