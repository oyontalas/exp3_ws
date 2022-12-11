#!/usr/bin/env python3

import time
import rospy
import numpy as np
import cv2
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

img = 0
re = 1
img_mask_corner = 0
pixwidth =  pixheight= 8
arrived_pixs = set()
goal_point = (100,100)
mx=8
my=8
end = 0
current_orientation = 0


def modify_goal():
    # 找到九宫格（九个瞭望点），对九个正方形计算范数。
    global goal_point
    global img
    rec_p1 = (goal_point[0] - 10, goal_point[1]-10)
    rec_p2 = (goal_point[0] + 10, goal_point[1]+10)
    rec_p3 = (goal_point[0] - 10, goal_point[1]+10)
    rec_p4 = (goal_point[0] + 10, goal_point[1]-10)
    
    rec_p5 = (goal_point[0] , goal_point[1]-10)
    rec_p6 = (goal_point[0] , goal_point[1]+10)
    rec_p7 = (goal_point[0] + 10, goal_point[1])
    rec_p8 = (goal_point[0] - 10, goal_point[1])
    
    s9 = img[goal_point[1]-5:goal_point[1]+5, goal_point[0]-5:goal_point[0]+5]
    
    s1 = img[rec_p1[1]-5:rec_p1[1]+5, rec_p1[0]-5:rec_p1[0]+5]
    s2 = img[rec_p2[1]-5:rec_p2[1]+5, rec_p2[0]-5:rec_p2[0]+5]
    s3 = img[rec_p3[1]-5:rec_p3[1]+5, rec_p3[0]-5:rec_p3[0]+5]
    s4 = img[rec_p4[1]-5:rec_p4[1]+5, rec_p4[0]-5:rec_p4[0]+5]
    
    s5 = img[rec_p5[1]-5:rec_p5[1]+5, rec_p5[0]-5:rec_p5[0]+5]
    s6 = img[rec_p6[1]-5:rec_p6[1]+5, rec_p6[0]-5:rec_p6[0]+5]
    s7 = img[rec_p7[1]-5:rec_p7[1]+5, rec_p7[0]-5:rec_p7[0]+5]
    s8 = img[rec_p8[1]-5:rec_p8[1]+5, rec_p8[0]-5:rec_p8[0]+5]
    
    s_lis = [s1, s2, s3, s4,s5,s6,s7,s8,s9]
    socre_lis = []
    for i in range(len(s_lis)):
        score = -np.max(s_lis[i]) + np.sum(np.ones_like(s1) == s_lis[i])
        socre_lis.append(score)
    # 选取最佳观察点
    idx = socre_lis.index(max(socre_lis))
    p_lis = [rec_p1, rec_p2, rec_p3, rec_p4, rec_p5, rec_p6, rec_p7, rec_p8, goal_point]
    goal_point = p_lis[idx]
    
def get_self_position(Odometry):
    # 获取自身坐标，将经过的轨迹加入哈希表
    global arrived_pixs
    global mx, my
    global current_orientation
    current_orientation = Odometry.pose.pose.orientation
    if re == 1:
        return
    x = -Odometry.pose.pose.position.x
    y = Odometry.pose.pose.position.y
    mx = (int)((pixwidth-x) /re)
    my = (int)(-(-pixheight-y) /re)
    cv2.circle(img_mask_corner, (mx, my),15,(255,0,0),2)
    for i in range(-18,18):
        for j in range(-18,18):
            arrived_pixs.add((mx+i, my+j))
            
    
def pix2map(mx, my):
    # 栅格地图坐标转换为世界坐标
    if re == 1:
        return
    x = (mx*re -pixwidth)
    y = my*re - pixheight
    return x, y


def get_global_map(OccupancyGrid):
    # map的回调函数， 根据现在的栅格地图计算角点，并与哈希表比对，更新目标点数组
    global img
    global re
    global img_mask_corner
    global goal_point
    global end
    try:
        
        re = OccupancyGrid.info.resolution
        data = OccupancyGrid.data
        data = np.array(data)
        data += 1
        data = np.array(data.reshape(OccupancyGrid.info.width, OccupancyGrid.info.height), dtype = np.uint8)
        img = cv2.cvtColor(data,cv2.COLOR_GRAY2BGR)
        img_change_tmp = deepcopy(img)
        img_change_tmp = cv2.GaussianBlur(img_change_tmp, (3, 3), 1.5) #去噪
        img_change_tmp=cv2.cvtColor(img_change_tmp, cv2.COLOR_RGB2GRAY)
        f = img_change_tmp.astype(np.float32)
        dst = cv2.cornerHarris(f, 3,3, 0.04)
        img_change_tmp = deepcopy(img)    
        x, y = np.where(dst>0.05*dst.max())
        # x, y都是列表，可能的目标点的两个维度
        goal_list = []
        for cx, cy in zip(x, y):
            if(cy, cx) in arrived_pixs: # 可视化，如果已经探索过就标记为蓝色
                cv2.circle(img_change_tmp, (cy, cx),1,[255,0,0])
                continue
                
            goal_list.append((cy, cx)) # 如果没有探索过就加入目标点列表
            img_change_tmp[cx, cy] = [0,255,0] # 待探索的目标标记为绿色
        img_mask_corner = img_change_tmp

        print(f'To Be Finished: {int(len(goal_list)*100/len(x))}%')
        if len(goal_list) == 0:
            print('finished - back now') # 目标列表为空表示都已经探索过了，该到出发点了
            goal_point = (int(OccupancyGrid.info.width/2), int(OccupancyGrid.info.height/2))
            end = 1
        else:
            dst_lis = []
            for i in range(len(goal_list)):
                x, y = pix2map(mx, my)
                gx, gy = pix2map(goal_list[i][0], goal_list[i][1])
                dst = (x-gx)**2 + (y-gy)**2
                dst_lis.append(dst)
            idx = dst_lis.index(min(dst_lis))
            goal_point = goal_list[idx]
            res = modify_goal()
            x, y = pix2map(goal_point[0], goal_point[1])
            print('current goal',(x,y))
    except Exception as e:
        print(e)

def show_img():
    while not rospy.is_shutdown():
        cv2.imshow('global map', img)
        cv2.imshow('v-graph explore', img_mask_corner)
        cv2.waitKey(1)


def goal_pose(point, end=0):
    goal_pose=MoveBaseGoal()
    goal_pose.target_pose.header.frame_id="map"
    goal_pose.target_pose.pose.position.x=point[0]
    goal_pose.target_pose.pose.position.y=point[1]
    goal_pose.target_pose.pose.position.z=0
 
    # r, p, y  欧拉角转四元数
    if end:
        x,y,z,w=tf.transformations.quaternion_from_euler(0,0,0)
 
        goal_pose.target_pose.pose.orientation.x=x
        goal_pose.target_pose.pose.orientation.y=y
        goal_pose.target_pose.pose.orientation.z=z
        goal_pose.target_pose.pose.orientation.w=w
    else:
        goal_pose.target_pose.pose.orientation = current_orientation
    return goal_pose


def mainThread():
    rospy.Subscriber("map", OccupancyGrid, get_global_map)
    rospy.Subscriber("odom", Odometry, get_self_position)
    rospy.spin()


def send_goal_point():
    last_time = time.time()
    last_goal = None
    while not rospy.is_shutdown():
        try:
            if time.time() < last_time+0.5:
                continue # 发一次阻塞一会，防止频发发布不同目标点导致抖动
            # 可视化，自己标注为白色，目标标注为红色
            cv2.circle(img_mask_corner, goal_point, 2, [0,0,255], 2) 
            cv2.circle(img_mask_corner, (mx, my), 2, [255,255,255], 2)
            if re == 1:
                continue
            if goal_point == (0.1, 0.1):
                continue

            
            x, y = pix2map(goal_point[0], goal_point[1])
            goal=goal_pose([x, y], end=end)
            if not last_goal is None and goal.target_pose.pose.position == last_goal.target_pose.pose.position:
                continue
            client.send_goal(goal)
            last_time = time.time()
            last_goal = goal
            print('send goal', x, y)
        except Exception as e:
            print(e)


if __name__ == '__main__':
    try:
        rospy.init_node('v_graph_explore', anonymous=False)
        client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        # drawThread = threading.Thread(target=show_img) # 可视化线程
        # drawThread.start()
        send_goal = threading.Thread(target=send_goal_point) # 发送目标点
        send_goal.start() 
        mainThread() # 主线程，订阅所需的信息，根据信息触发其他操作
        
    except Exception as e:
        print(e)