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

# g_x = [-0.9602898565, -0.7966664774, -0.5255324099, -0.1834346425, 0.1834346425, 0.5255324099, 0.7966664774, 0.9602898565]
# g_a = [0.1012285363, 0.2223810345, 0.3137066459, 0.3626837834, 0.3626837834, 0.3137066459, 0.2223810345, 0.1012285363]
g_x = [-0.9782286581460570, -0.8870625997680950, -0.7301520055740490, -0.5190961292068110, -0.2695431559523440, 0.0000000000000000, 0.2695431559523440, 0.5190961292068110, 0.7301520055740490, 0.8870625997680950, 0.9782286581460570]
g_a = [0.0556685671161736, 0.1255803694649040, 0.1862902109277340, 0.2331937645919900, 0.2628045445102460, 0.2729250867779000, 0.2628045445102460, 0.2331937645919900, 0.1862902109277340, 0.1255803694649040, 0.0556685671161736]
# g_x = []
# g_a = []
# GL = [-0.9915651684209300, 0.0216160135264833, -0.9558239495713970, 0.0497145488949698, -0.8926024664975550, 0.0764257302548890, -0.8037049589725230, 0.1009420441062870, -0.6916870430603530, 0.1225552067114780, -0.5597708310739470, 0.1406429146706500, -0.4117511614628420, 0.1546846751262650, -0.2518862256915050, 0.1642764837458320, -0.0847750130417353, 0.1691423829631430, 0.0847750130417353, 0.1691423829631430, 0.2518862256915050, 0.1642764837458320, 0.4117511614628420, 0.1546846751262650, 0.5597708310739470, 0.1406429146706500, 0.6916870430603530, 0.1225552067114780, 0.8037049589725230, 0.1009420441062870, 0.8926024664975550, 0.0764257302548890, 0.9558239495713970, 0.0497145488949698, 0.9915651684209300, 0.0216160135264833]
# for i in range(len(GL) / 2):
#     g_x.append(GL[2 * i])
#     g_a.append(GL[2 * i + 1])


def solveProblem():
    print_time = rospy.Time().now().to_sec()

    U = ca.SX.sym('U', 2 * len(g_a) + 1)
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
    det_t = U[-1] / 2 * g_x[0] + U[-1] / 2
    xf[4] += det_t * U[1]
    xf[3] += det_t * U[0]
    xf[2] += det_t * xf[4]
    xf[1] += det_t * U[0] * ca.sin(xf[2])
    xf[0] += det_t * U[0] * ca.cos(xf[2])
    for i in range(len(g_x) - 1):
        det_t = U[-1] / 2 * (g_x[i + 1] - g_x[i])
        xf[4] += det_t * U[2 * i + 3]
        xf[3] += det_t * U[2 * i + 2]
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
    for i in range(len(g_x)):
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

    points = PointCloud()
    points.header.frame_id = 'map'
    UU = res['x']
    xf[0] = x0[0]
    xf[1] = x0[1]
    xf[2] = x0[2]
    xf[3] = x0[3]
    xf[4] = x0[4]
    points.points.append(Point32(xf[0], xf[1], 0))
    det_t = ca.SX.sym('det_t', 1)
    det_t = UU[-1] / 2 * g_x[0] + UU[-1] / 2
    xf[4] += det_t * UU[1]
    xf[3] += det_t * UU[0]
    xf[2] += det_t * xf[4]
    xf[1] += det_t * UU[0] * ca.sin(xf[2])
    xf[0] += det_t * UU[0] * ca.cos(xf[2])
    points.points.append(Point32(xf[0], xf[1], 0))
    for i in range(len(g_x) - 1):
        det_t = UU[-1] / 2 * (g_x[i + 1] - g_x[i])
        xf[4] += det_t * UU[2 * i + 3]
        xf[3] += det_t * UU[2 * i + 2]
        xf[2] += det_t * xf[4]
        xf[1] += det_t * xf[3] * ca.sin(xf[2])
        xf[0] += det_t * xf[3] * ca.cos(xf[2])
        points.points.append(Point32(xf[0], xf[1], 0))
    g_pub_clothoid.publish(points)
    #while not rospy.is_shutdown():
    for i in range(100000):
        g_pub_clothoid.publish(points)
        #rospy.Duration(0.1)
    print('end pub')
    

if __name__ == '__main__':
    print('opt control test.')
    rospy.init_node("opt_control_test", anonymous = True)
    solveProblem()
    rospy.spin()