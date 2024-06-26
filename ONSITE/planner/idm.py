
#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
# 版本号:V2.6.14
# 更新日期:2023.5.30
# 更新内容：
    # 调整道路边界读取，适应C卷
    # 增加道路角度识别，用于指示回正角度
    # 修改变道许可中非终点道路bug
    # 修改无障碍上下偏设定值，提升舒适度
    # 增加终点区域判定方法，以兼容C卷环境
    # 增加场景最大车辆速度读取方法，以求更契合场景的期望速度
    # 取消道路剩余长度判定，默认叠加100以取消算法中对道路剩余长度的判定，基于动态规划更合理的使用道路
    # 此版本适用于基本段
    # 避障减速处的期望距离问题
    # 调整侧向避让加速区间，大于10后匀速驾驶
    # 调整适应场景的最大车速，提升舒适度
    # 调整变道策略中，上下可变道识别，增强C卷适应度，调整上下可变区间为终点线1.5米范围
    # 调整避撞全加速，超过对象车10速不再过度加速
    # 调整避让判定，与对象车纵向距离足以无碰撞避让
    # 调整加速以及期望速度，进行取整和余量控制，提升舒适度
    # 修改道路宽度识别bug，完善道路宽度取值位置
    # 调整所有分母防止为0错误
    # 其他车辆数据清零时，维持本车速度，优化舒适度
    # 修改道路角度读取办法，改为实时读取前方的小角度

class IDM():
    def __init__(self, a_bound=15, exv=40, t=1.2, a=3, b=2.4, gama=2, s0=2.0, s1=2.0): # V1.6 考虑exv值调大，不然加速度上不来
        """跟idm模型有关的模型参数，一定要记得调整

        :param a_bound: 本车加速度绝对值的上下界
        :param exv: 期望速度
        :param t: 反应时间
        :param a: 起步加速度
        :param b: 舒适减速度
        :param gama: 加速度指数
        :param s0: 静止安全距离
        :param s1: 与速度有关的安全距离选择参数
        """
        self.a_bound = a_bound
        self.exv = exv
        self.t = t
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0

    # 主调用函数，向 main返回下一步操作
    def act(self, observation, luwang_list, time_test_start, traj=None):
        frame = pd.DataFrame()
        for key, value in observation['vehicle_info'].items():
                sub_frame = pd.DataFrame(value,columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[key])
                # print('sub_frame数据：',sub_frame)
                if traj:
                    if key != 'ego':    # 对于背景车
                        t = round(float(observation['test_setting']['t']),2)   # 当前时间
                        dt = round(float(observation['test_setting']['dt']),2) # 时间步长
                        t_n = round(t + dt , 2) # 下步时间
                        try:
                            x = traj[key][str(t)]['x']  # 当前x位置
                            x_n = traj[key][str(t_n)]['x']  # 下步x位置
                            sub_frame['v'] = abs((x_n-x)/dt) # 计算背景车速度
                        except KeyError:    # 缺失背景车数据
                            sub_frame['v'] = 0

                frame = pd.concat([frame, sub_frame])
                # print('frame数据：',frame)
        state = frame.to_numpy()
        # 取出车道数量值
        lane_num = self.lanes_num(luwang_list)

        lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, lane_width = self.lane_info(luwang_list, lane_num, state)
        # 取出当前场景其他车辆的最大速度
        exv_max_other =  max(state[1:,2])
        if exv_max_other < 15:
            exv_max_ego = exv_max_other * 1.2
        elif 15 <= exv_max_other < 30:
            exv_max_ego = exv_max_other + 10
        else:
            exv_max_ego = 40
        print(exv_max_other, '当前场景其他车辆最大速度')
        if exv_max_other == 0:
            exv_max_ego = state[0, 2]
        self.exv = int(exv_max_ego)

        # print('state数据：',state)
        # 判断正反向驾驶场景，以及当前最近的回正角度
        driving_orgin, yaw_center = self.pi_near(state) 
        # 将本车 x值与所有中心线起始 x值比较，列出可行驶中心线列表
        lane_center_dring_list, lane_center_dring_yaw_list = self.deside_lane_center_drving_list(driving_orgin, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list)
        # 在可行使道路列表中，计算出当前的道路角度
        a_lane_yaw = 0
        b_lane_yaw = []
        for i in lane_center_dring_list:
            a_lane_yaw = np.absolute(state[0, 1] - i)
            # print(final_y_a)
            b_lane_yaw.append(a_lane_yaw)
        # print(final_y_b)
        c_lane_yaw_index = b_lane_yaw.index(min(b_lane_yaw))
        lane_now_yaw = lane_center_dring_yaw_list[c_lane_yaw_index]
        print('当前道路的角度为', lane_now_yaw)

        # 在可行使道路列表中，筛选出与目的最近的中心线
        final_y_a = 0
        final_y_b = []
        for i in lane_center_dring_list:
            final_y_a = np.absolute((observation['test_setting']['goal']['y'][1] - observation['test_setting']['goal']['y'][0])/2 + observation['test_setting']['goal']['y'][0] - i)
            final_y_b.append(final_y_a)
        final_y_c = final_y_b.index(min(final_y_b))
        final_y_goal = lane_center_dring_list[final_y_c]
        
        a_idm, final_y_acc, lane_center_dring_distance = self.deside_acc(state,lane_center, driving_orgin, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, observation, final_y_goal, lane_now_yaw, exv_max_ego, lane_width)
        yaw, final_y, deside_avoid_acc, change_acc, yaw_0= self.deside_yaw(observation, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, final_y_acc, lane_now_yaw, lane_width)
        
        # 增加针对目的区域的方向判断
        yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, final_y, 0.6, observation)
        # 存放自然判定的加速度值
        a_idm_acc = a_idm
        # 进行避障加减速判定
        # final_y_acc_up, final_y_acc_down, change_direction = self.deside_change(state, lane_center, lane_center_x_max, lane_center_x_min)
        # final_y, avoid_allow, deside_avoid_acc = self.deside_avoid(state, final_y, final_y_acc_up, final_y_acc_down)
        v_1, fv_1, dis_gap_1,direction_1, a_car_num_1 = self.getInformFront(state, 1, lane_now_yaw)
        v_2, fv_2, dis_gap_2,direction_2, a_car_num_2 = self.getInformFront(state, -1, lane_now_yaw)
        # self.s_ = self.s0 + np.absolute(v - fv) * (np.absolute(v - fv) / self.a_bound + 1)
        self.s_1 = self.s0 + v_1 * ((v_1 - fv_1) / self.a_bound ) + state[0, 4] # 期望前车距离
        self.s_2 = self.s0 + v_2 * ((fv_2 - v_1) / self.a_bound ) + state[0, 4] # 期望后车距离
        print('当前测试时刻', time_test_start * observation['test_setting']['dt'])

        
        # print("最匹配的中心线为：", final_y_goal, state[0,0])
        print(deside_avoid_acc, change_acc, np.absolute(state[0, 1] - final_y_goal))
        if (deside_avoid_acc == 0.5 or( change_acc == 0.5 and deside_avoid_acc == 0 and 
        (np.absolute(y_gap_true) > 0.4)) 
        and (dis_gap_1 > self.s_1 or dis_gap_1 == -1)): 
            a_idm = 0
            print('避障匀速通过')
        elif (((deside_avoid_acc == 1 or( change_acc == 1 and deside_avoid_acc == 0 and 
        (np.absolute(y_gap_true) > 0.4)))
        and (dis_gap_1 > self.s_1 or dis_gap_1 == -1)) or (deside_avoid_acc == 1.1) or (deside_avoid_acc == 1.2)): # 加速避障或者后向拥塞
            # a_idm = 20*self.a * (1 - (v / self.exv) ** self.gama)
            # print('deside_avoid_acc,change_acc',deside_avoid_acc,change_acc)
            if (deside_avoid_acc == 1.2 or ( change_acc == 1 and deside_avoid_acc == 0 and 
        (np.absolute(y_gap_true) > 0.4))):
                a_idm = 20*self.a * (1 - (v_1 / (v_1 + 10)) ** self.gama)
            elif fv_2 >= 40 and np.absolute(v_1 - 40) > 2 :
                self.exv =int(fv_2 + 3)
                # a_idm =  (fv_2 - v_2) * self.a * (1 - (v_2 / self.exv) ** self.gama)
                a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama + ((self.s_2 / (dis_gap_2+1e-6)) ** 2))
                self.exv = exv_max_ego # 计算完加速度后，调回原值，避免传递到下一个方法
                print('当前避障加速度1',a_idm)
            elif 0 < fv_2 < 40 and v_1 < 35:
                self.exv = exv_max_ego
                # a_idm =  (fv_2 - v_2) * self.a * (1 - (v_2 / self.exv) ** self.gama)
                a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama + ((self.s_2 / (dis_gap_2+1e-6)) ** 2))
                print('当前避障加速度2',a_idm, fv_2, v_2)
            elif v_1 >= 35: # 已到速度上限，仍有加速需求时
                self.exv = 50
                a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama + ((self.s_2 / (dis_gap_2+1e-6)) ** 2))
                # a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama)
                self.exv = exv_max_ego # 计算完加速度后，调回原值，避免传递到下一个方法
                print('当前避障加速度3',a_idm)
            else:
                self.exv = exv_max_ego
                # a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama)
                a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama + ((self.s_2 / (dis_gap_2+1e-6)) ** 2))
                print('当前避障加速度4',a_idm)
            # 取自然加速度和避障加速度的最大值
            if a_idm_acc > a_idm:
                a_idm = a_idm_acc
        
        elif ((deside_avoid_acc == -1 or (change_acc == -1 and deside_avoid_acc == 0 and 
        (np.absolute(y_gap_true) > 0.4) 
        and (dis_gap_1 > self.s_1 or dis_gap_1 == -1))) or (deside_avoid_acc == -1.1)): # 检测到避障减速指令或前向拥塞时，执行避障减速策略
            dis_gap_lateral_list = []
            for i in state[1:, 0]:
                dis_gap_lateral_list.append(np.absolute(i - state[0, 0]))
            dis_gap_lateral = min(dis_gap_lateral_list)
            a_dis_gap_lateral = dis_gap_lateral_list.index(dis_gap_lateral)
            dis_gap_lateral_s_ = self.s0 + v_1 * (np.absolute(v_1 - state[a_dis_gap_lateral, 2]) / self.a_bound) + state[0, 4]
            a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama - ((dis_gap_lateral_s_ / (dis_gap_lateral+1e-6)) ** 2)) # V1.8 调整减速度参数期望距离，提升减速性能
            if a_idm_acc < a_idm:
                a_idm = a_idm_acc
            print(deside_avoid_acc, change_acc,'1111')
            print('当前避障减速度',a_idm)
        elif deside_avoid_acc == 3: # 夹持加速
            if v_1 > fv_2:
                a_idm = 0
                print('夹持后向匀速')
            else:
                a_idm = self.a * (1 - (v_1 / (fv_2 + 10)) ** self.gama)
                print('当前夹持避障加速度',a_idm)
        elif deside_avoid_acc == -3: # 夹持减速
            if v_1 < fv_1:
                a_idm = 0
                print('夹持匀前向速')
            else:
                a_idm = self.a * (1 - (v_1 / (fv_1 + 1e-6)) ** self.gama)
                print('当前夹持避障减速度',a_idm)
                if a_idm_acc < a_idm:
                    a_idm = a_idm_acc
        elif deside_avoid_acc == -1.2: 
            a_idm = -20*self.a * (1 - (v_1 / self.exv) ** self.gama)

        if lane_center_dring_distance < 50 and a_idm >= 0: # 剩余可行使距离不到20减速
            # a_idm = -self.a * (1 - (v_1 / self.exv) ** self.gama)
            a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama - ((20/ (lane_center_dring_distance+1e-6)) ** 2)) # V1.8 调整减速度参数期望距离，提升减速性能
            print('当前道路尽头减速度',a_idm)
        # 对加速度进行约束
        a_idm = np.clip(a_idm, -self.a_bound, 15)
        
        # if (v_1 < 0.5 and a_idm < 0) or (yaw != yaw_0):
        # if (v_1 < 1  and a_idm < 0): # 速度低到0.5时不再减速，避免速度为0跳出测试
        #     a_idm = 0
        print('当前速度', v_1)
        print('当前加速度', a_idm)

        # 如果可行使距离为0，立即向最近可行驶道路偏转，兼容初始车辆即在道路外的场景
        if lane_center_dring_distance == 0:
            a_lane_center_drving_list = -1 
            a_lane_center_drving_list_min = []
            for i in lane_center_dring_list:
                a_lane_center_drving_list += 1
                a_lane_center_drving_list_min.append(np.absolute(i - state[0, 1]))
            a_index = a_lane_center_drving_list_min.index(min(a_lane_center_drving_list_min))
            if driving_orgin:
                if lane_center_dring_list[a_index] > state[0, 1]: # 根据最近可行使中心线位置进行偏转
                    yaw = np.arctan((yaw_center -  0.174533 - state[0, 3]) * state[0, 4] / 1.7 / (state[0, 2] * round(float(observation['test_setting']['dt']),2 + 1e-6)))  # 上偏
                else:
                    yaw = np.arctan((yaw_center +  0.174533 - state[0, 3]) * state[0, 4] / 1.7 / (state[0, 2] * round(float(observation['test_setting']['dt']),2 + 1e-6)))  # 下偏
            else:
                if lane_center_dring_list[a_index] > state[0, 1]: # 根据最近可行使中心线位置进行偏转
                    yaw = np.arctan((yaw_center +  0.174533 - state[0, 3]) * state[0, 4] / 1.7 / (state[0, 2] * round(float(observation['test_setting']['dt']),2 + 1e-6)))  # 上偏
                else:
                    yaw = np.arctan((yaw_center -  0.174533 - state[0, 3]) * state[0, 4] / 1.7 / (state[0, 2] * round(float(observation['test_setting']['dt']),2 + 1e-6)))  # 下偏
        # print(self.exv,"self.exv")
        return a_idm, yaw
    
    # 加速度计算方法
    def deside_acc(self, state,lane_center, driving_orgin, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, observation, final_goal_y, lane_now_yaw, exv_max_ego, lane_width):
        # 实例化可行驶中心线列表
        lane_center_dring_list, lane_center_dring_yaw_list =self.deside_lane_center_drving_list(driving_orgin, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list)
        # 记录当前行驶中心线剩余可行驶距离是否大于30米
        lane_center_dring_distance = self.deside_lane_center_drving_distance(state, state[0, 1], lane_center_dring_list, lane_center, lane_center_x_max, lane_center_x_min, driving_orgin)
        # 判断是否具有变道条件
        final_y_acc_up, final_y_acc_down, change_refuse_list = self.deside_change(state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, lane_center_dring_distance, final_goal_y, observation, lane_now_yaw, lane_width)
        # print('超车中可行驶中心线列表',laneChange)
        final_y_acc = False
        v_1, fv_1, dis_gap_1,direction_1, a_car_num_1 = self.getInformFront(state, 1, lane_now_yaw)
        v_2, fv_2, dis_gap_2,direction_2, a_car_num_2 = self.getInformFront(state, -1, lane_now_yaw)
        # 如果剩余场景时间以当前速度无法完成，则将安全距离缩短参数调整至100
        # 增加针对目的区域的方向判断
        yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, final_goal_y, 0.6, observation)

        # 安全距离
        if (np.absolute(y_gap_true) < 0.6 and dis_gap_1 > 0 and 
        ((driving_orgin == True and (state[0, 0] - observation['test_setting']['goal']['x'][1]) < 100) or 
        (driving_orgin == False and ( observation['test_setting']['goal']['x'][1]) - state[0, 0] < 100))):
            self.s_1 = v_1 * ((v_1 - fv_1) / self.a_bound) + self.s0  # 期望距离改为1米
            print('近终点与前车期望安全距离为：', self.s_1)
        else:
            self.s_1 = self.s0 + v_1 * ((v_1 - fv_1) / self.a_bound) + state[0, 4]
            print('与前车期望安全距离为：', self.s_1)
        # print(v, fv, dis_gap,direction)
        # print(state)
        # if dis_gap < 0 or dis_gap > 30 or v < fv-0.5: # 与前车相距100米开外、与前车相距20米开外、小于前车速度, 速度接近上限时匀速驾驶
        print('剩余可行驶距离',lane_center_dring_distance)
        if ((v_1 <  fv_1 - 0.5 or dis_gap_1 == -1 or dis_gap_1 > self.s_1) and lane_center_dring_distance >= 20)  :
            if fv_1 > 30:
                self.exv = int(fv_1) + 10
            # if dis_gap_1 == -1:
            #     self.exv = v_1 + 1
            if (dis_gap_1 > self.s_1 + 5 or (dis_gap_1 == -1)) and np.abs(v_1 - self.exv) > 1: # 与前车在期望距离外较大加速度，期望距离内控制加速度
                a_idm = 20 * self.a * (1 - (v_1 / self.exv) ** self.gama)
                self.exv = exv_max_ego
                print('当前加速度1',a_idm)
                # print(self.a, v, self.exv, self.gama)
            # elif np.abs(state[0, 2] - fv) < 1 or (dis_gap - self.s_ < 1 and v > fv):
            elif v_1 + 0.5 > fv_1:
                a_idm = 0
                self.exv = exv_max_ego
                print('当前匀速')
            else:
                a_idm = self.a * (1 - (v_1 / self.exv) ** self.gama)
                self.exv = exv_max_ego
                print('当前加速度2',a_idm)
                # print(self.a, v, self.exv, self.gama)
            # print('exv',self.exv)
        # elif dis_gap > self.s_ and (0 < self.exv - v < 1): # 前车距离大于期望距离，且本车速度接近上限值，进行匀速驾驶
        #     a_idm = 0
        #     print('')
        # elif 0 < fv - v <= 0.5 and 1 < dis_gap <= 15 : # 本车与前车速度相近时，采取匀速策略
        #     a_idm = 0
        elif np.absolute(v_1 - fv_1) <= 0.5 and dis_gap_1 > 5:
            a_idm = 0
        else:
         # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            # self.s_ = self.s0 + max(self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
            #     v - fv) / 2 / (self.a * self.b) ** 0.5,0) 
            
            # a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
            a_idm = 15 * self.a * (1 - (v_1 / self.exv) ** self.gama - ((self.s_1 / (dis_gap_1 + 1e-6)) ** 2)) # V1.8 调整减速度参数期望距离，提升减速性能
            print('当前减速度',a_idm,dis_gap_1)
            # if dis_gap  < 15 and np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) > 300: # 与前车相距小于15米，并且距离终点大于200米内时，激活超车策略
            # 与前车相距小于15米，并且距离终点大于200米内时，激活超车策略
            if np.absolute(y_gap_true) > 1 : # 终点线外开启超车
                # print('启动超车模块')
                if dis_gap_1 < 15 and dis_gap_1 != -1:
                    if final_goal_y > state[0, 1] and final_y_acc_up:
                        final_y_acc = state[0, 1] + 2
                    elif final_goal_y < state[0, 1] and final_y_acc_down:
                        final_y_acc = state[0, 1] - 2
        
        # 对加速度进行约束
        # a_idm = np.clip(a_idm, -self.a_bound/3, 1e7)
        a_idm = np.clip(a_idm, -self.a_bound, 15) # V1.8 增大加速度最小值约束，使本车具有更好的减速性能
        # print(v,fv,dis_gap,a_idm,self.s_)
        # print(state,v,fv,dis_gap,a_idm)
        # print(a_idm, final_y_acc)
        return a_idm, final_y_acc, lane_center_dring_distance
    # 跟驰模型，前车信息获取方法
    def getInformFront(self, state, ego_other, lane_now_yaw):
        # direction = np.sign(state[0,2])
        driving_orgin, pi_nearest = self.pi_near(state)
        if driving_orgin: # V1.4 更新场景判别方法
            direction = -1.0 # -1代表正向驾驶场景
        else:
            direction = 1.0 # 1代表反向驾驶场景
        # state[:,0] = state[:,0]*direction 
        # state[:,2] = state[:,2]*direction
        ego = state[0,:] # 取出本车所有信息
        v, fv, dis_gap = ego[2], -1, -1
        
        # V1.6增加行驶场景方向判断
        driving_orgin, pi_nearest = self.pi_near(state)
        if driving_orgin:
            # 正向驾驶场景
            if ego_other == 1:
                x_ind = ego[0] + ego[4]/2 * np.cos(lane_now_yaw) > state[:,0] + state[:,4]/2 * np.cos(lane_now_yaw)
            elif ego_other == -1:
                x_ind = ego[0] + ego[4]/2 * np.cos(lane_now_yaw) < state[:,0] + state[:,4]/2 * np.cos(lane_now_yaw)
        else:           
            # 反向驾驶场景
            # x_ind = ego[0] < state[:,0] # 判断所有车是否在本车前侧
            if ego_other == 1:
                x_ind = ego[0] + ego[4]/2 * np.cos(lane_now_yaw) < state[:,0] + state[:,4]/2 * np.cos(lane_now_yaw)
            elif ego_other == -1:
                x_ind = ego[0] + ego[4]/2 * np.cos(lane_now_yaw) > state[:,0] + state[:,4]/2 * np.cos(lane_now_yaw)
        # 求解两车纵向间距时，增加道路角度判定，以支持在斜路上行驶
        gap_ego_y = np.cos(lane_now_yaw) * ((np.abs(ego[1] - state[:,1])) - np.tan(lane_now_yaw) * (np.abs(ego[0] - state[:,0])))
        y_ind = gap_ego_y < ((ego[5] + state[:,5])/2) # 判断所有车与本车纵向距离是否小于两车宽度的一半，即以此判断是否是正前方车辆
        ind = x_ind & y_ind
        # print('在本车前侧',ego[0],state[:,0],x_ind, y_ind, ind, ind.sum())
        if ind.sum() > 0:
            state_ind = state[ind,:] # 同车道所有前车列表
            front = state_ind[np.absolute((state_ind[:,0]-ego[0])).argmin(),:] # 同车道前车离本车最近的前车所有信息
            # print('111',state_ind,front,state_ind[:,0],np.absolute((state_ind[:,0]-ego[0])).argmin())
            fv = front[2] # 最近同车道前车速度
            gap_ego_x = np.absolute(front[0] - ego[0]) / (np.cos(lane_now_yaw) + 1e-6)
            dis_gap = (gap_ego_x - (ego[4] + front[4])/2) # 本车与同车道最近前车横向距离
        # 增加一个根据前后车距离判定车辆编号的方法，便于开发测试
        a_car_num = 0
        if dis_gap != -1:
            for i in state[1:, :]:
                a_car_num += 1
                # if (np.absolute(state[0, 0] - i) - (state[0, 4] + state[a_car_num, 4]) / 2 )- dis_gap <= 0.1:
                # print(i, front)
                if i[0] == front[0] and i[1] == front[1]:
                    if ego_other == 1: # 前方车辆
                        # print('最近前车',a_car_num)
                        break
                    elif ego_other == -1: # 后方车辆
                        # print('最近后车',a_car_num)
                        break
        return v, fv, dis_gap, direction, a_car_num

    # 方向盘转角计算方法
    def deside_yaw(self, observation, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, final_y_acc, lane_now_yaw, lane_width):
        #确定目标区域
        if observation['test_setting']['goal']['y'][0] > observation['test_setting']['goal']['y'][1]:
            goal_y_min = observation['test_setting']['goal']['y'][1]
            goal_y_max = observation['test_setting']['goal']['y'][0]
        else:
            goal_y_min = observation['test_setting']['goal']['y'][0]
            goal_y_max = observation['test_setting']['goal']['y'][1]
        # 变道加速度参数
        change_acc = 0
        # 当前ego偏航角
        ego_yaw_now = state[0, 3]
        # print(ego_yaw_now)
        # 车辆轴距
        ego_l = state[0, 4] / 1.7
        # 时间步长
        ego_dt = round(float(observation['test_setting']['dt']),2) 
        # 当前速度
        ego_v_now = state[0, 2]
        # 当前本车 x值
        ego_x_now = state[0, 0]
        # print('本车x值', ego_x_now, state[0, 0])

        # 选择ego质心落在终点中心线区间
        final_space = 0.6
        # 纵向偏移量设定为0.4米
        ego_y_offset = 0.4
        # 纵向速度
        ego_y_v = ego_y_offset/ego_dt
        # 车辆偏航角
        if ego_v_now > ego_y_v:
            ego_yaw = np.arcsin(ego_y_v/(ego_v_now + 1e-6))
            ego_yaw = np.clip(ego_yaw, 0, 0.174533) # 限制最大角度到30°
            # print('车辆偏航角', ego_y_v, ego_dt, ego_v_now, ego_yaw)
        else:
            ego_yaw = 0.174533

        # 判断正反向驾驶场景，以及当前最近的回正角度
        driving_orgin, yaw_center = self.pi_near(state)
        # 记录回正方向盘转角，用于变道时将加速度置0
        yaw_0 = np.arctan((yaw_center-ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
        # 将本车 x值与所有中心线起始 x值比较，列出可行驶中心线列表
        lane_center_dring_list, lane_center_dring_yaw_list = self.deside_lane_center_drving_list(driving_orgin, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list)
        
        # 记录当前行驶中心线剩余可行驶距离是否大于30米
        lane_center_dring_distance = self.deside_lane_center_drving_distance(state, state[0, 1], lane_center_dring_list, lane_center, lane_center_x_max, lane_center_x_min, driving_orgin)
        yaw_center = yaw_center + lane_now_yaw
        # V1.6修复ego车超出目标区域判断BUG，将该判断与正反向场景判定关联
        # 在可行使道路列表中，筛选出与目的最近的中心线
        final_y_a = 0
        final_y_b = []
        # print((goal_y_max - goal_y_min)/2 + goal_y_min)
        # print('可行驶中心线列表：',lane_center_dring_list)
        for i in lane_center_dring_list:
            final_y_a = np.absolute((goal_y_max - goal_y_min)/2 + goal_y_min - i)
            # print(final_y_a)
            final_y_b.append(final_y_a)
        # print(final_y_b)
        final_y_c = final_y_b.index(min(final_y_b))
        final_goal_y = lane_center_dring_list[final_y_c]
        # print(lane_center_dring_list,'可行驶道路列表',final_goal_y)
        if final_goal_y  > goal_y_max - 1:
            final_goal_y = (goal_y_max - goal_y_min) / 2 + goal_y_min
        elif final_goal_y < goal_y_min + 1:
            final_goal_y = (goal_y_max - goal_y_min) / 2 + goal_y_min
        if driving_orgin: 
            # V1.5 如果本车超出终点，将不再刷新中心线列表，中心线置为终点的中心 y值，以此来兼容驶出场景状态
            if state[0, 0] < observation['test_setting']['goal']['x'][1]:
                final_y = final_goal_y
                print('ego车已驶出目标区域')
            else:
                final_y = final_goal_y
                print("final区域中心线为：", final_y)
        else:
            # V1.5 如果本车超出终点，将不再刷新中心线列表，中心线置为终点的中心 y值，以此来兼容驶出场景状态
            if state[0, 0] > observation['test_setting']['goal']['x'][1]:
                final_y = final_goal_y
                print('ego车已驶出目标区域')
            else:
                final_y = final_goal_y
                print("final区域中心线为：", final_y, state[0,0])
        # 判断是否具有变道条件
        final_y_acc_up, final_y_acc_down, change_refuse_list = self.deside_change(state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, lane_center_dring_distance, final_goal_y, observation, lane_now_yaw, lane_width)
        # print(final_y_acc_up,final_y_acc_down,'final_y_acc_up,final_y_acc_down')
        # 判断是否需要超车
        if final_y_acc:
            final_y = final_y_acc
            print('超车中心线为', final_y)
        # 离终点只有30米的时候，优先终点中心线
        if 0 < np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) <= 50:
            final_y = final_goal_y
            print('ego车与终点相距%s米' % np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]))
        # print('final_y', final_y)
        # 侧向避障检测
        final_y, avoid_allow, deside_avoid_acc = self.deside_avoid(state, final_y, final_y_acc_up, final_y_acc_down, lane_center_dring_distance, final_goal_y, observation, change_refuse_list, lane_now_yaw)
        # print('avoid_allow', avoid_allow)
        # print(state[0, 1],(final_y - final_space),(final_y + final_space))
        # print('目标上限',goal_y_max, '目标下限',goal_y_min,(goal_y_max - goal_y_min)/2 + goal_y_min)
        # 增加针对目的区域的方向判断
        yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, final_y, final_space, observation)
        # 增加针对目的区域的方向判断
        if driving_orgin:
            print("正向驾驶场景")
            # print('final_y', final_y)
            # print(state[0, 1], (final_y - final_space), (final_y + final_space))
            # 判断是否落在目标中心线中心区域：在中心区域执行回正，在区域上方执行下偏，在区域下方执行上偏
            if yaw_hold:
                yaw = np.arctan((yaw_center-ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print("回正值:",yaw_center)
            elif yaw_down:
                # if (final_y_acc_down == 1 or final_y_acc or np.absolute(final_y - state[0, 1]) < 2 )and (avoid_allow not in  [10, 20, -20]): # 变道许可 or 超车许可 or 本车所属中心道路内可变(侧向无碰撞风险)
                if (final_y_acc_down == 1 or final_y_acc or (2 > y_gap_true > 1))and ((avoid_allow not in  [10, 10.5, 20, -20, 1.1, -1.1]) or lane_center_dring_distance <= 100):    
                    yaw = np.arctan((yaw_center + ego_yaw -ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
                    change_acc = 0.5 # 执行匀速
                    print('下偏值：',yaw_center + ego_yaw, state[0, 1])
                else:
                    yaw = np.arctan((yaw_center - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
                    print("不满足下变道条件，回正值:", yaw_center, state[0, 1])
                    if 2 in change_refuse_list: # 下前方拥塞 
                        change_acc = -1 # 执行减速
                        print('下前方拥塞执行减速')
                    elif -2 in change_refuse_list: # 下后方拥塞 
                        change_acc = 1 # 执行加速 
                        print('下后方拥塞执行加速')
                    elif 3.2 in change_refuse_list: #  近距离拥塞加速
                        change_acc = 1 # 执行加速 
                        print('下方侧向近距离拥塞执行加速')
                    elif -3.2 in change_refuse_list: #  近距离拥塞减速
                        change_acc = -1 # 执行减速 
                        print('下方侧向近距离拥塞执行减速')
                    elif 0.5 in change_refuse_list: #  近距离拥塞匀速
                        change_acc = 0.5 # 执行匀速 
                        print('下方侧向近距离拥塞执行匀速')
            elif yaw_up:
                # 变道许可 or 超车许可 or 本车所属中心道路内可变(侧向无碰撞风险、无前后向碰撞风险)
                # if (final_y_acc_up == 1 or final_y_acc or np.absolute(final_y - state[0, 1]) < 2) and (avoid_allow not in  [-10, 20, -20]): 
                if (final_y_acc_up == 1 or final_y_acc or (-2 < (y_gap_true) < -1)) and ((avoid_allow not in  [-10, 10.5, 20, -20, 1.1, -1.1]) or lane_center_dring_distance <= 100): 
                    yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                    change_acc = 0.5 # 执行匀速
                    print('上偏值：',yaw_center - ego_yaw, state[0, 1])
                else:
                    yaw = np.arctan((yaw_center - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
                    print("不满足上变道条件，回正值:", yaw_center, state[0, 1])
                    if 1 in change_refuse_list: # 上前方拥塞 
                        change_acc = -1 # 执行减速
                        print('上前方拥塞执行减速')
                    elif -1 in change_refuse_list: # 上后方拥塞 
                        change_acc = 1 # 执行加速 
                        print('上后方拥塞执行加速')
                    elif 3.1 in change_refuse_list: #  近距离拥塞加速
                        change_acc = 1 # 执行加速 
                        print('上方侧向近距离拥塞执行加速')
                    elif -3.1 in change_refuse_list: #  近距离拥塞减速
                        change_acc = -1 # 执行减速 
                        print('上方侧向近距离拥塞执行减速')
                    elif 0.5 in change_refuse_list: #  近距离拥塞匀速
                        change_acc = 0.5 # 执行匀速 
                        print('上方侧向近距离拥塞执行匀速') 
            if avoid_allow == 1 or avoid_allow == 1.1: # 侧向防撞激活
                yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防侧向碰撞,上偏值：', yaw_center - ego_yaw, state[0, 1])
            elif avoid_allow == -1 or avoid_allow == -1.1: # 侧向防撞激活
                yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防侧向碰撞,下偏值：', yaw_center + ego_yaw, state[0, 1])
            # 后向防撞激活
            if avoid_allow == -2.1: # 满足上变道
                yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防后向碰撞,上偏值：',yaw_center - ego_yaw, state[0, 1])
            elif avoid_allow == -2.2: # 满足下变道
                yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防后向碰撞,下偏值：', yaw_center + ego_yaw, state[0, 1])
            # 前向防撞激活
            if avoid_allow == 2.1: # 满足上变道
                yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防前向碰撞,上偏值：', yaw_center - ego_yaw, state[0, 1])
            elif avoid_allow == 2.2: # 满足下变道
                yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防前向碰撞,下偏值：', yaw_center + ego_yaw, state[0, 1])
        else:
            print("反向驾驶场景")
            if yaw_hold:
                yaw = np.arctan((yaw_center - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                # print(yaw_center,ego_yaw_now,ego_l,ego_v_now,ego_dt)
                print("回正值:", yaw_center)
            elif yaw_down:
                # 变道许可 or 超车许可 or 本车所属中心道路内可变(侧向无碰撞风险)
                # if (final_y_acc_down == 1 or final_y_acc or np.absolute(final_y - state[0, 1]) < 2) and (avoid_allow not in  [10, 20, -20]):
                if (final_y_acc_down == 1 or final_y_acc or (2 > y_gap_true > 1)) and ((avoid_allow not in  [10, 10.5, 20, -20, 1.1, -1.1]) or lane_center_dring_distance <= 100 ):   
                    yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
                    change_acc = 0.5 # 执行匀速
                    print('下偏值：', yaw_center - ego_yaw)
                else:
                    yaw = np.arctan((yaw_center - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
                    print("不满足下变道条件，回正值:", yaw_center)
                    if 2 in change_refuse_list: # 下前方拥塞 
                        change_acc = -1 # 执行减速
                        print('下前方拥塞执行减速')
                    elif -2 in change_refuse_list: # 下后方拥塞 
                        change_acc = 1 # 执行加速 
                        print('下后方拥塞执行加速')
                    elif 3.2 in change_refuse_list: #  近距离拥塞加速
                        change_acc = 1 # 执行加速 
                        print('侧向近距离拥塞执行加速')
                    elif -3.2 in change_refuse_list: #  近距离拥塞减速
                        change_acc = -1 # 执行减速 
                        print('侧向近距离拥塞执行减速')
                    elif 0.5 in change_refuse_list: #  近距离拥塞匀速
                        change_acc = 0.5 # 执行匀速 
                        print('侧向近距离拥塞执行匀速')  
            elif yaw_up :
                # 变道许可 or 超车许可 or 本车所属中心道路内可变(无碰撞风险)
                if (final_y_acc_up == 1  or final_y_acc or (-2 < (y_gap_true) < -1)) and ((avoid_allow not in  [-10, 10.5, 20, -20, 1.1, -1.1]) or lane_center_dring_distance <= 100):  
                    yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                    change_acc = 0.5 # 执行匀速
                    print('上偏值：', yaw_center + ego_yaw, '当前y值', state[0, 1])
                    # print(final_y_acc_up, final_y_acc, final_y, final_space)
                else:
                    yaw = np.arctan((yaw_center - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6))
                    print("不满足上变道条件，回正值:", yaw_center)
                    # print(change_refuse_list)
                    if 1 in change_refuse_list: # 上前方拥塞 
                        change_acc = -1 # 执行减速
                        print('上前方拥塞执行减速')
                    elif -1 in change_refuse_list: # 上后方拥塞 
                        change_acc = 1 # 执行加速 
                        print('上后方拥塞执行加速')
                    elif 3.1 in change_refuse_list: #  近距离拥塞加速
                        change_acc = 1 # 执行加速 
                        print('侧向近距离拥塞执行加速')
                    elif -3.1 in change_refuse_list: #  近距离拥塞减速
                        change_acc = -1 # 执行减速 
                        print('侧向近距离拥塞执行减速')
                    elif 0.5 in change_refuse_list: #  近距离拥塞匀速
                        change_acc = 0.5 # 执行匀速 
                        print('侧向近距离拥塞执行匀速') 
            if avoid_allow == 1 or avoid_allow == 1.1:
                yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防侧向碰撞,上偏值：', yaw_center + ego_yaw)
            elif avoid_allow == -1 or avoid_allow == -1.1:
                yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防侧向碰撞,下偏值：', yaw_center - ego_yaw)
            if avoid_allow == -2.1: # 满足上变道
                yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防后向碰撞,上偏值：', yaw_center + ego_yaw, state[0, 1])
            elif avoid_allow == -2.2: # 满足下变道
                yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防后向碰撞,下偏值：', yaw_center - ego_yaw, state[0, 1])
            if avoid_allow == 2.1: # 满足上变道
                yaw = np.arctan((yaw_center + ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防前向碰撞,上偏值：',yaw_center + ego_yaw, state[0, 1])
            elif avoid_allow == 2.2: # 满足下变道
                yaw = np.arctan((yaw_center - ego_yaw - ego_yaw_now) * ego_l / (ego_v_now * ego_dt + 1e-6)) 
                print('防前向碰撞,下偏值：', yaw_center - ego_yaw, state[0, 1],)
        # print(final_y,'final_y')
        return yaw, final_y, deside_avoid_acc, change_acc, yaw_0

    # 判断当前车辆朝向的最近回正值
    def pi_near(self, state):
        pi_nearest = 0
        yaw_init = state[0, 3]
        # 车辆朝向初始记为True
        driving_orgin = True
        i = 0
        while True:
            if yaw_init >= 0:
                if np.absolute(yaw_init - i * np.pi) <= np.pi/2:
                    pi_nearest = i * np.pi
                    break
                else:
                    i += 1
            else:
                if np.absolute(yaw_init + i * np.pi) <= np.pi/2:
                    pi_nearest = i * np.pi
                    break
                else:
                    i += 1
        # 如果整数位是偶数则返回False，表明位反向驾驶，否则返回True表明为正向驾驶
        if (int(pi_nearest)) % 2 == 0:
            driving_orgin = False
        # print("当前驾驶方向、回正角度：", driving_orgin, pi_nearest)
        return driving_orgin, pi_nearest

    # 判断是否具备变道条件（检测相邻车道前后7米是否有车辆）
    def deside_change(self, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, lane_center_dring_distance, final_goal_y, observation, lane_now_yaw, lane_width): 
        change_refuse_list= [] # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
        driving_orgin, pi_nearest  = self.pi_near(state)
        # 在可行使车道中心线中寻找可变道车道
        final_y_acc = False
        final_y_acc_up = 0 # 判断可借道中心线在本车的左右位置
        final_y_acc_down = 0
        laneChange, lane_center_dring_yaw_list = self.deside_lane_center_drving_list(driving_orgin, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list)
        chang_allow_list = [] # V1.5 修改判断可借道中心线方法Bug
        change_list = [] # 可变道中心线列表
        # 增加针对目的区域的方向判断
        yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, final_goal_y, 0.6, observation)
        v_1, fv_1, dis_gap_1,direction_1, a_car_num_1 = self.getInformFront(state, 1, lane_now_yaw) # 取出最近前车数据
        v_2, fv_2, dis_gap_2,direction_2, a_car_num_2 = self.getInformFront(state, -1, lane_now_yaw) # 取出最近后车数据
        if (np.absolute(state[0, 1] - final_goal_y) < 0.4 and dis_gap_1 > 0 and 
        ((driving_orgin == True and ( 0 < state[0, 0] - observation['test_setting']['goal']['x'][1]) < 100) or 
        (driving_orgin == False and (0 < observation['test_setting']['goal']['x'][1]) - state[0, 0] < 100))):
            self.s_1 = v_1 * ((v_1 - fv_1) / self.a_bound) + self.s0  # 期望距离改为1米
        else:
            self.s_1 = self.s0 + v_1 * ((v_1 - fv_1) / self.a_bound ) + state[0, 4] # 期望前车距离
        self.s_2 = self.s0 + v_2 * ((v_2 - fv_2) / self.a_bound ) + state[0, 4] # 期望后车距离
        for i in laneChange: # 在可行驶中心线中寻找目标
            y_gap_i = self.deside_ego_now_gap(state, lane_now_yaw, i, 0)
            chang_allow_list = [] # V1.5 修改判断可借道中心线方法Bug
            # print(y_gap_i, i, lane_width, state[0, 5])
            if  (np.absolute(y_gap_i) - state[0, 5]/2)  <= lane_width: # 判断 i中心线与本车纵向距离是否小于4米,确保是向相邻车道借道:
                # print('laneChange,i',laneChange,i)
                a = 0
                for j in state[1:,1]: # 取出其他车 y值
                    a += 1
                    # 取出该车横纵距离
                    gap_ego_x, gap_ego_y = self.deside_ego_gap(state, lane_now_yaw, a)
                    ego_other_y = self.deside_ego_location(state, lane_now_yaw, a)
                    # print('lane_width', lane_width)
                    y_gap_i = self.deside_ego_now_gap(state, lane_now_yaw, i, a)
                    if (np.absolute(y_gap_i) - state[a, 5]/2) <= (lane_width / 2) :  # 判断 j车是否在 i中心道路上
                        # print(a,i,'车落在该道路上',(np.absolute(state[0, 0] - state[a, 0]) - (state[0, 4] + state[a, 4])/2), final_goal_y)
                        # 正向驾驶场景, a车在本车前方，且本车速度大于 a车，变速距离小于实际距离，; a车在本车后方，且本车速度小于 a车,变速距离小于实际距离
                        # 正向驾驶场景, a车在本车前方，且本车速度小于等于a车,且间距大于0 ; a车在本车后方，且本车速度大于等于 a车，且间距大于0
                        # 反向驾驶场景, a车在本车前方，且本车速度大于 a车,变速距离小于实际距离; a车在本车后方，且本车速度小于 a车,变速距离小于实际距离 
                        # 反向驾驶场景, a车在本车前方，且本车速度小于 a车，且间距大于0; a车在本车后方，且本车速度大于 a车，且间距大于0   
                        # 可变道至少要有一个车身距离
                        if ((i != final_goal_y ) and ((((driving_orgin == True) and (((state[0, 0] > state[a, 0]) and (state[0, 2] > state[a, 2]) 
                        and (np.absolute(v_1 - state[a, 2]) / self.a_bound * (v_1 + 1e-6)) < (gap_ego_x))
                        or ((state[0, 0] < state[a, 0]) and (state[0, 2] < state[a, 2]) and 
                        (np.absolute(v_1 - state[a, 2]) / self.a_bound * (state[a, 2] + 1e-6)) < (gap_ego_x))
                        or ((state[0, 0] > state[a, 0]) and (state[0, 2] <= state[a, 2]) and (gap_ego_x))
                        or ((state[0, 0] < state[a, 0]) and (state[0, 2] >= state[a, 2]) and (gap_ego_x))))
                        or ((driving_orgin == False) and (((state[0, 0] < state[a, 0]) 
                        and (state[0, 2] > state[a, 2]) and ((np.absolute(v_1 - state[a, 2]) / self.a_bound * (v_1 + 1e-6)) < (gap_ego_x)))
                        or ((state[0, 0] > state[a, 0]) and (state[0, 2] < state[a, 2]) 
                        and ((np.absolute(v_1 - state[a, 2]) / self.a_bound * (state[a, 2] + 1e-6)) < (gap_ego_x)))
                        or ((state[0, 0] < state[a, 0]) and (state[0, 2] <= state[a, 2]) and (gap_ego_x))
                        or ((state[0, 0] > state[a, 0]) and (state[0, 2] >= state[a, 2]) and (gap_ego_x)))))
                        and (lane_center_dring_distance > 100) and( (gap_ego_x) > state[0, 4]))):
                            chang_allow_list.append(True)
                            # if a == 10:
                                # print(a,(np.absolute(state[0, 0] - state[a, 0]) - (state[0, 4] + state[a, 4])/2),(np.absolute(state[0, 1] - state[a, 1]) - (state[0, 5] + state[a, 5])/2))
                                # print(state[0,0], state[a,0],state[0,2],state[a,2])
                        # 当判断终点所在道路时，降低对前后车的判定，以求进入终点车道,有1.5个车身长度即可变道
                        # elif ((i == final_goal_y and np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) < 30) 
                        elif (((i == final_goal_y or (((final_goal_y < i < state[0, 1]) == True) or ((state[0, 1] < i < final_goal_y) == True))) or lane_center_dring_distance <= 100) 
                        and ((((driving_orgin == True) and (((state[0, 0] > state[a, 0]) and (state[0, 2] > state[a, 2]) 
                        and (gap_ego_x) > 2 * (np.absolute(state[a, 2] - state[0, 2]) * (4 / (state[0, 2] * 0.17365 + 1e-6))))
                        or ((state[0, 0] < state[a, 0]) and (state[0, 2] < state[a, 2]) and 
                        (gap_ego_x) > 2 * (np.absolute(state[a, 2] - state[0, 2]) * (4 / (state[0, 2] * 0.17365 + 1e-6))))
                        or ((state[0, 0] > state[a, 0]) and (state[0, 2] <= state[a, 2]) and ((gap_ego_x) > 2))
                        or ((state[0, 0] < state[a, 0]) and (state[0, 2] >= state[a, 2]) and ((gap_ego_x) > 2))))
                        or ((driving_orgin == False) and (((state[0, 0] < state[a, 0]) and (state[0, 2] > state[a, 2]) 
                        and ((gap_ego_x) > np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * state[0, 2]))
                        or ((state[0, 0] > state[a, 0]) and (state[0, 2] < state[a, 2]) 
                        and ((gap_ego_x) > 2 * (np.absolute(state[a, 2] - state[0, 2]) * (4 / (state[0, 2] * 0.17365 + 1e-6)))))
                        or ((state[0, 0] < state[a, 0]) and (state[0, 2] <= state[a, 2]) and ((gap_ego_x) > 2))
                        or ((state[0, 0] > state[a, 0]) and (state[0, 2] >= state[a, 2]) and ((gap_ego_x) > 2)))))
                        and (lane_center_dring_distance > 100) and((gap_ego_x) > 
                        (np.absolute(state[a, 2] - state[0, 2]) * (4 / (state[0, 2] * 0.17365 + 1e-6)))))):
                            chang_allow_list.append(True)
                        
                        # 如果剩余距离不足100，放宽变道条件，以兼容汇入汇出场景
                        elif lane_center_dring_distance <= 100:
                            if (gap_ego_x) > state[0, 4]:
                                chang_allow_list.append(True)
                                # print(a,'111',(np.absolute(state[0, 0] - state[a, 0]) - (state[0, 4] + state[a, 4]) / 2))
                            else:
                                chang_allow_list.append(False)
                        else:
                            if (gap_ego_x) > state[0, 4]: # 与本车至少有5米横向距离
                                if driving_orgin: # 正向驾驶场景
                                    if state[0, 0] > state[a, 0]: # a车在本车前方
                                        if state[0, 2] <= state[a, 2]: # 前车速度大于等于本车
                                            # final_y_acc = i # 如果 i可行驶车道上有 j车辆，但 j车辆与本车大于7米，执行变道，将变道目标中心线传递给变道方法
                                            chang_allow_list.append(True)
                                        else: # 前车速度小于本车
                                            # 如果可安全减速前车距离小于实际前车距离，则可安全变道
                                            if (np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) < (gap_ego_x):
                                                chang_allow_list.append(True)
                                            else:
                                                chang_allow_list.append(False)
                                                # print(a,'在本车前方邻车道7米内')
                                                if np.absolute(y_gap_true) > 0.6: # 不在终点所在道路执行拥塞检测
                                                    if state[a, 1] > ego_other_y: # 在本车上方
                                                        change_refuse_list.append(1) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                        # print(a,'在本车上前方拥塞')
                                                    else:
                                                        change_refuse_list.append(2) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                        # print(a,'在本车下前方拥塞')
                                    else: # a车在本车后方
                                            if state[0, 2] >= state[a, 2]: # 后车速度小于等于本车
                                                chang_allow_list.append(True)
                                            else: # 后车速度大于本车
                                                # 如果可安全减速前车距离小于实际后车距离，则可安全变道
                                                if (np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) < (gap_ego_x):
                                                    chang_allow_list.append(True)
                                                    # print('%s车在目标中心线%s ,后车速度大于本车' % (a, i))
                                                else:
                                                    chang_allow_list.append(False)
                                                    # print(a,'在本车后方邻车道7米内')
                                                    if np.absolute(y_gap_true) > 0.6: # 不在终点所在道路执行拥塞检测
                                                        if state[a, 1] > ego_other_y: # 在本车上方
                                                            change_refuse_list.append(-1) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                        else:
                                                            change_refuse_list.append(-2) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                            
                                else: # 反向驾驶场景
                                    if state[0, 0] < state[a, 0]: # a车在本车前方
                                        if state[0, 2] <= state[a, 2]: # 前车速度大于等于本车
                                            # final_y_acc = i # 如果 i可行驶车道上有 j车辆，但 j车辆与本车大于7米，执行变道，将变道目标中心线传递给变道方法
                                            chang_allow_list.append(True)
                                            # print('%s车在目标中心线%s 7米外' % (a, i))
                                        else: # 前车速度小于本车
                                            # 如果可安全减速前车距离小于实际前车距离，则可安全变道
                                            if (np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) < (gap_ego_x):
                                                chang_allow_list.append(True)
                                                # print('%s车在目标中心线%s ,后车速度大于本车' % (a, i))
                                            else:
                                                chang_allow_list.append(False)
                                                # print(a,'在本车前方邻车道7米内')
                                                if np.absolute(y_gap_true) > 0.6: # 不在终点所在道路执行拥塞检测
                                                    if state[a, 1] > ego_other_y: # 在本车上方
                                                        change_refuse_list.append(1) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                        # print(a,'在本车上前方拥塞')
                                                    else:
                                                        change_refuse_list.append(2) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                    
                                                        # print(a,'在本车下前方拥塞')
                                    else: # a车在本车后方
                                            if state[0, 2] >= state[a, 2]: # 后车速度小于等于本车
                                                chang_allow_list.append(True)
                                                # print('%s车在目标中心线%s 7米外' % (a, i))
                                            else: # 后车速度大于本车
                                                # 如果可安全减速前车距离小于实际后车距离，则可安全变道
                                                if (np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) < (gap_ego_x):
                                                    chang_allow_list.append(True)
                                                    # print('%s车在目标中心线%s ,后车速度大于本车' % (a, i))
                                                else:
                                                    chang_allow_list.append(False)
                                                    # print(a,'在本车后方邻车道7米内')
                                                    if np.absolute(y_gap_true) > 0.6: # 不在终点所在道路执行拥塞检测
                                                        if state[a, 1] > ego_other_y: # 在本车上方
                                                            change_refuse_list.append(-1) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                            # print(a,'在本车上后方拥塞')
                                                        else:
                                                            change_refuse_list.append(-2) # 1代表本车上前方拥塞 -1上后方拥塞 2下前方拥塞 -2下后方拥塞
                                                            # print(a,'在本车下后方拥塞')
                                                        # if  state[a, 2] - state[0, 2] > 10: # 防止过度减速
                                                        #     chang_allow_list = []
                                                        #     change_refuse_list.append(0.5)
                            else:
                                chang_allow_list.append(False) # 与本车不足横向5米
                                if np.absolute(y_gap_true) > 0.6: # 不在终点所在道路执行拥塞检测
                                    if state[0, 1] < state[a, 1]: # 在本车上方
                                        if ((( (state[0, 2] > state[a, 2]) )and lane_center_dring_distance > 100 
                                        and state[0, 2] - state[a, 2] < 5) or (dis_gap_2 <= self.s_2 and dis_gap_2 != -1)): # 速度低于本车时且剩余道路大于100时或者后向距离不足时，加速创造条件
                                            change_refuse_list.append(3.1) # 3代表侧向近距离车，拥塞加速
                                        elif np.absolute(state[a, 2] - state[0, 2]) < 5: # 小于拥塞车速度10即可，防止过度减速
                                            change_refuse_list.append(-3.1) # -3代表侧向近距离车，拥塞减速
                                            # print(a, i,'与本车不足横向5米')
                                        else:
                                            change_refuse_list.append(0.5) # 0.5代表匀速
                                    else:
                                        if ((( (state[0, 2] > state[a, 2]) )and lane_center_dring_distance > 100 
                                        and state[0, 2] - state[a, 2] < 5) or (dis_gap_2 <= self.s_2 and dis_gap_2 != -1)): # 速度低于本车时且剩余道路大于100时或者后向距离不足时，加速创造条件
                                            change_refuse_list.append(3.2) # 3代表侧向近距离车，拥塞加速
                                        elif np.absolute(state[a, 2] - state[0, 2]) < 5: # 小于拥塞车速度10即可，防止过度减速
                                            change_refuse_list.append(-3.2) # -3代表侧向近距离车，拥塞减速
                                            # print(a, i,'与本车不足横向5米')
                                        else:
                                            change_refuse_list.append(0.5) # 0.5代表匀速
            else:
                chang_allow_list.append(False) # 非相邻车道
                # print('在非相邻车道')
            if False not in chang_allow_list and chang_allow_list != []: 
                final_y_acc = i
                # print('chang_allow_list, i',chang_allow_list, i)
                # 将本车 x值与所有中心线起始 x值比较，列出可行驶中心线列表
                lane_center_dring_list, lane_center_dring_yaw_list = self.deside_lane_center_drving_list(driving_orgin, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list)
                # 记录当前行驶中心线剩余可行驶距离是否大于30米
                lane_center_dring_distance = self.deside_lane_center_drving_distance(state, final_y_acc, lane_center_dring_list, lane_center, lane_center_x_max, lane_center_x_min, driving_orgin)
                
                if lane_center_dring_distance > 121:
                    # # 增加针对目的区域的方向判断
                    # yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, i, 0.6, observation)
                    # 增加针对目的区域的方向判断
                    # yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, i, 0.6, observation)
                    y_gap_now = self.deside_ego_now_gap(state, lane_now_yaw, i, 0)
                    if np.absolute(y_gap_now) > 0.2:
                        # if i + 1 > state[0, 1] + state[0, 5]/2: # 本车上边框小于中心线上边界，可向上偏移 
                        if (y_gap_now < 0) or (y_gap_now + state[0, 5]/2 < 1 and y_gap_now > 0): # 车在终点线下方或者车上边界不超过终点线上方1米
                            final_y_acc_up = 1
                            # print(y_gap_true , y_gap_true + state[0, 5]/2, '上变道条件检测')
                        # elif i - 1 < state[0, 1] - state[0, 5]/2: # 本车下边框大于中心线下边界，可向下偏移
                        elif (y_gap_now > 0) or (state[0, 5]/2 - y_gap_now < 1 and y_gap_now < 0): # 车在终点线上方或者车下边界不超过终点线下方1米
                            final_y_acc_down = 1
                    # print("超车借道目标中心线为：", final_y_acc, state[0,1])
                    change_list.append(final_y_acc)
                    
        if final_y_acc:
            # change_allow_num = len(change_list)
            if final_y_acc_up + final_y_acc_down == 2: # 可变道中心线至少与本车有0.2米距离，避免在本车道晃动
                print("上下均可变道")
            elif final_y_acc_up == 1:
                print('可向上变道')
            elif final_y_acc_down == 1:
                print('可向下变道')
        else:
            print("无可变道中心线")
        return final_y_acc_up, final_y_acc_down, change_refuse_list

    # 碰撞检测算法 V1.8增加前向防碰撞检测、增加防撞加减速策略
    def deside_avoid(self, state, final_y, final_y_acc_up, final_y_acc_down, lane_center_dring_distance, final_goal_y, observation, change_refuse_list, lane_now_yaw):
        driving_orgin, pi_nearest = self.pi_near(state)
        v_1, fv_1, dis_gap_1,direction_1, a_car_num_1 = self.getInformFront(state, 1, lane_now_yaw) # 取出最近前车数据
        v_2, fv_2, dis_gap_2,direction_2, a_car_num_2 = self.getInformFront(state, -1, lane_now_yaw) # 取出最近后车数据
        print('最近前车', a_car_num_1, '距离', dis_gap_1, '速度', fv_1)
        # print('最近前车速度', fv_1)
        print('最近后车', a_car_num_2, '距离', dis_gap_2, '速度', fv_2)
        # self.s_ = self.s0 + np.absolute(v - fv) * (np.absolute(v - fv) / self.a_bound + 1)
        # y_gap_true = self.deside_ego_now_gap(state, lane_now_yaw, final_goal_y, 0)
        # 增加针对目的区域的方向判断
        yaw_up, yaw_down, yaw_hold, y_gap_true = self.deside_ego_final(state, lane_now_yaw, final_goal_y, 0.6, observation)
        if (np.absolute(y_gap_true) < 0.6 and dis_gap_1 > 0 and 
        ((driving_orgin == True and (0 < state[0, 0] - observation['test_setting']['goal']['x'][1]) < 100) or 
        (driving_orgin == False and (0 < observation['test_setting']['goal']['x'][1]) - state[0, 0] < 100))):
            self.s_1 = v_1 * ((v_1 - fv_1) / self.a_bound) + self.s0  # 期望距离改为1米
        else:
            self.s_1 = self.s0 + v_1 * ((v_1 - fv_1) / self.a_bound ) + state[0, 4] # 期望前车距离
        self.s_2 = self.s0 + v_2 * ((v_2 - fv_2) / self.a_bound ) + state[0, 4] # 期望后车距离
        avoid_allow = 0
        deside_avoid_acc = 0
        # exv_up = 0
        a = 0
        a_lateral_avoidance = [] # 检测是否同时存在上下侧向碰撞风险
        a_clamp_avoidance = [] # 检测是否存在前后同时碰撞风险
        a_lateral_avoidance_up = False # 检测上侧风险来车距离，以评定优先级 
        a_lateral_avoidance_down = False # 检测下侧风险来车距离，以评定优先级 
        a_lateral_avoidance_fomer = False # 检测前向风险来车距离，以评定优先级 
        a_lateral_avoidance_after = False # 检测后向风险来车距离，以评定优先级 
        # 检测所有其他车辆
        for i in state[1:, 0]:
            a += 1
            # 取出该车横纵距离
            gap_ego_x, gap_ego_y = self.deside_ego_gap(state, lane_now_yaw, a)
            ego_other_y = self.deside_ego_location(state, lane_now_yaw, a)
            # if np.absolute(i - state[0, 0]) < 7: # i车与本车横向距离小于7米
            if gap_ego_x < 50: # 检测 i车辆与本车横向距离 
                # print(a,'该车横向与本车小于15米')
                # print(a, '该车纵向与本车距离为：', (np.absolute(state[a, 1] - state[0, 1]) - (state[a, 5] + state[0, 5])/2))
                # 侧向避障算法
                if gap_ego_x < 0: # 规避较长车辆的侧向碰撞风险，车辆较长时容易出现两车横向距离为负，纵向距离为负的情况
                    y_distance = -1
                else:
                    y_distance = 0
                if 1 > gap_ego_y > y_distance: # 检测 i车辆与本车纵向距离，增加纵向车辆速度判定，避免过度加速
                    # print(a,'纵向距离较近')
                # 纵向距离判断更改为
                    # 正向驾驶场景, a车在本车前方，且本车速度大于 a车; a车在本车后方，且本车速度小于 a车 并且本车与a车之间没有其他车辆
                    # 反向驾驶场景, a车在本车前方，且本车速度大于 a车; a车在本车后方，且本车速度小于 a车 并且本车与a车之间没有其他车辆
                    # 检测1米内，来得及减速就减速，来不及就加速
                    if ((((driving_orgin == True) and ((state[0, 0] > state[a, 0] and state[0, 2] > state[a, 2] 
                    and ((dis_gap_1 > gap_ego_x) or (dis_gap_1 == -1)))
                     or (state[0, 0] < state[a, 0] and state[0, 2] < state[a, 2] 
                     and ((dis_gap_2 > gap_ego_x) or (dis_gap_2 == -1))))) or 
                    ((driving_orgin == False) and((state[0, 0] < state[a, 0] and state[0, 2] > state[a, 2] 
                    and ((dis_gap_1 > gap_ego_x) or (dis_gap_1 == -1))) 
                    or ((state[0, 0] > state[a, 0]) and (state[0, 2] < state[a, 2])
                    and ((dis_gap_2 > gap_ego_x) or (dis_gap_2 == -1))))) or 
                    (gap_ego_x < 0))
                    and (np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) - (gap_ego_x)) > 10):
                        print(a, '该车横、纵向与本车距离为,有侧向碰撞风险', gap_ego_x, gap_ego_y)
                        lateral_avoid_allow = (gap_ego_x > 0) or ((gap_ego_x < 0) and (gap_ego_y > (state[0, 4]/2 * np.sin(0.174533))))  # 横向间距大于0， 或者纵向间距大于车身偏移量，才可执行避让
                        # 如果 i车在本车下方，则将当前目标中心线上拉，并且没有强制避让情况，且无更优先级的下侧向风险
                        if ((state[a, 1] < ego_other_y) and (avoid_allow != (1.1 and -1.1)) and 
                        (a_lateral_avoidance_down == False or (np.absolute(state[0, 0] - state[a, 0]) < a_lateral_avoidance_down))): 
                            a_lateral_avoidance.append(-1) # 碰撞风险参数,-1代表存在下侧碰撞风险
                            a_lateral_avoidance_down = np.absolute(state[0, 0] - state[a, 0]) # 将与 a车的距离存放，若再次出现侧向风险，以此选定优先级
                            # 终点在上方
                            if y_gap_true < -0.6 and final_y_acc_up == 1 and lateral_avoid_allow:
                                final_y = final_y + 1
                                avoid_allow = 1
                                # deside_avoid_acc = 0.5 
                                print('上避让1',a)
                                print('当前y值，中心线值',state[0, 1],final_y)
                            # 终点在下方
                            elif ((np.absolute(y_gap_true) > 1 and final_y_acc_down == 1 
                            and ((((state[0, 0] < state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] > state[a, 0]) and (driving_orgin == False)))
                            or((((state[0, 0] > state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] < state[a, 0]) and (driving_orgin == False))) 
                            and (np.absolute(v_1 - state[a, 2]) < 10)))) and lateral_avoid_allow):     
                                final_y = final_y - 1
                                avoid_allow = -1
                                # deside_avoid_acc = 0.5 
                                print('下避让',a)
                                print('当前y值，中心线值',state[0, 1],final_y)  
                            elif (1 in a_lateral_avoidance) and (-1 in a_lateral_avoidance): # 上下均有侧向风险，加速过
                                if (dis_gap_1 > self.s_1 or dis_gap_1 == -1) and np.absolute(y_gap_true) > 1 and ((1 or 2) not in change_refuse_list):
                                    avoid_allow = 10.5 # 10.5代表上下均有风险
                                    deside_avoid_acc = 1 
                                else:   
                                    avoid_allow = 10.5 # 10.5代表上下均有风险
                                    deside_avoid_acc = -1 
                            elif final_y_acc_up == 1 and lateral_avoid_allow: # 有上避让条件
                                final_y = final_y + 1
                                avoid_allow = 1
                                # deside_avoid_acc = 0.5 
                                print('上避让2',a)
                                print('当前y值，中心线值',state[0, 1],final_y) 
                            else:
                                if (((state[0, 0] > state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] < state[a, 0]) and (driving_orgin == False))
                                and state[a, 2] - state[0, 2] < 5): # 在本车前方
                                    print(a, '在本车前方')
                                    if ((gap_ego_x < 0 
                                    and ( (0 < state[a, 2] - state[0, 2] < 10))) and (-1 not in a_clamp_avoidance)):
                                        print('无上避让条件，全减速通过')
                                        avoid_allow = 10
                                        deside_avoid_acc = -1.2 # -1.2代表全减速通过
                                    else:
                                        if gap_ego_x < 0 and (dis_gap_1 > self.s_1 or dis_gap_1 == -1) and (state[0, 2] > state[a, 2]):
                                            print('无下避让条件，全加速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = 1.2
                                        elif ((np.absolute(v_1 - state[a, 2]) / self.a_bound * (v_1 + 1e-6)) < (gap_ego_x)
                                        and (-1 not in a_clamp_avoidance) and state[a, 2] - state[0, 2] < 5):
                                            print('无上避让条件，减速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = -1
                                        elif (((np.absolute(v_1 - state[a, 2]) / self.a_bound * (v_1 + 1e-6)) < (gap_ego_x)
                                        and (-1 not in a_clamp_avoidance) and state[a, 2] - state[0, 2] >= 5) or ((state[0, 2] - state[a, 2] > 10) and (-1 not in a_clamp_avoidance))) :
                                            print('无上避让条件，匀速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = 0.5
                                        else:  
                                            print('无上避让条件，加速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = 1
                                elif ((state[0, 0] < state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] > state[a, 0]) and (driving_orgin == False)): # 在本车后方
                                    print(a, '在本车后方')
                                    # 与本车很近，且前向车辆距离较近，没有加速条件
                                    if ((gap_ego_x < 0 
                                    and ((0 < state[a, 2] - state[0, 2] < 10))) and (-1 not in a_clamp_avoidance)):
                                        print('无上避让条件，全减速通过')
                                        avoid_allow = 10
                                        deside_avoid_acc = -1.2 # -1.2代表全减速通过
                                    else:
                                        if gap_ego_x < 0 and (dis_gap_1 > self.s_1 or dis_gap_1 == -1) and (state[0, 2] > state[a, 2]):
                                            print('无下避让条件，全加速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = 1.2
                                        elif ((np.absolute(v_1 - state[a, 2]) / self.a_bound * (state[a, 2] + 1e-6)) < (gap_ego_x)
                                        and dis_gap_1 > self.s_1):
                                            print('无上避让条件，加速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = 1
                                        elif state[a, 2] - state[0, 2] >= 5 and (self.s_1 < dis_gap_1 or dis_gap_1 == -1):
                                            print('无上避让条件，匀速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = 0.5
                                        else:  
                                            print('无上避让条件，减速通过')
                                            avoid_allow = 10
                                            deside_avoid_acc = -1    
                            if  ((gap_ego_y < 0.3) 
                            and (0 < gap_ego_x < 1) and ((1 in a_lateral_avoidance) and (-1 in a_lateral_avoidance)) == False and lateral_avoid_allow): # 纵向距离小于0.2且横向距离小于0强制上偏
                                final_y = final_y + 1
                                avoid_allow = 1.1 # 1.1代表强制上避让
                                deside_avoid_acc = 0.5 
                                print('强制上避让',a)
                                print('当前y值，中心线值',state[0, 1],final_y)    
                        # 如果 i车在本车上方，则将当前目标中心线下拉，并且没有强制避让情况，且无更优先级的上侧向风险
                        elif ((state[a, 1] > ego_other_y) and (avoid_allow != (1.1 and -1.1)) and 
                        (a_lateral_avoidance_up == False or (np.absolute(state[0, 0] - state[a, 0]) < a_lateral_avoidance_up))):      
                            a_lateral_avoidance.append(1) # 碰撞风险参数,1代表存在上侧碰撞风险
                            a_lateral_avoidance_up = np.absolute(state[0, 0] - state[a, 0])
                            # 终点在上方
                            if ((y_gap_true < -0.6 and final_y_acc_up == 1 
                            and ((((state[0, 0] < state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] > state[a, 0]) and (driving_orgin == False)))
                            or((((state[0, 0] > state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] < state[a, 0]) and (driving_orgin == False))) 
                            and (np.absolute(v_1 - state[a, 2]) < 10)))) and lateral_avoid_allow): 
                            # # 终点在上方
                            # if final_goal_y > state[0, 1] and final_y_acc_up == 1:
                                final_y = final_y + 1
                                avoid_allow = 1
                                # deside_avoid_acc = 0.5 
                                print('上避让3',a)
                                print('当前y值，中心线值',state[0, 1],final_y)
                            # 终点在下方
                            elif y_gap_true > 0.6 and final_y_acc_down == 1 and lateral_avoid_allow:     
                                final_y = final_y - 1
                                avoid_allow = -1
                                # deside_avoid_acc = 0.5 
                                print('下避让',a)
                                print('当前y值，中心线值',state[0, 1],final_y) 
                            elif (1 in a_lateral_avoidance) and (-1 in a_lateral_avoidance): # 上下均有侧向风险，减速过
                                print('上下均有碰撞风险')
                                if (dis_gap_1 > self.s_1 or dis_gap_1 == -1) and np.absolute(y_gap_true) > 1 and ((1 or 2) not in change_refuse_list):
                                    avoid_allow = 10.5 # 10.5代表上下均有风险
                                    deside_avoid_acc = 1
                                else:
                                    avoid_allow = 10.5 # 10.5代表上下均有风险
                                    deside_avoid_acc = -1
                            elif  (final_y_acc_down == 1 ) and lateral_avoid_allow: # 有下避让条件
                                final_y = final_y - 1
                                avoid_allow = -1
                                # deside_avoid_acc = 0.5 
                                print('下避让',a)
                                print('当前y值，中心线值',state[0, 1],final_y)
                            
                            else:
                                if ((state[0, 0] > state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] < state[a, 0]) and (driving_orgin == False)): # 在本车前方
                                    print(a, '在本车前方')
                                    # 如果有前向碰撞风险，全减速通过
                                    if ((((gap_ego_x < 0) 
                                    and ((0 < state[a, 2] - state[0, 2] < 10) or (dis_gap_1 - self.s_1 < 3 * state[0, 4])))) and (-1 not in a_clamp_avoidance)):
                                        print('无下避让条件，全减速通过')
                                        avoid_allow = -10
                                        deside_avoid_acc = -1.2 # -1.2代表全减速通过
                                    else:
                                        # 变速距离满足且后向距离允许采取减速策略  
                                        if gap_ego_x < 0 and (dis_gap_1 > self.s_1 or dis_gap_1 == -1) and (state[0, 2] > state[a, 2]):
                                            print('无下避让条件，全加速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = 1.2
                                        elif (((np.absolute(v_1 - state[a, 2]) / self.a_bound * (v_1 + 1e-6)) < gap_ego_x)
                                        and (self.s_2 + 5 < dis_gap_2 or dis_gap_2 == -1) and (state[a, 2] - state[0, 2] < 10)):     
                                            print('无下避让条件，减速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = -1
                                        elif (((np.absolute(v_1 - state[a, 2]) / self.a_bound * (v_1 + 1e-6)) < gap_ego_x
                                        and (self.s_2  + 5 < dis_gap_2 or dis_gap_2 == -1) and (np.absolute(v_1 - state[a, 2]) >= 10)) or ((state[0, 2] - state[a, 2] > 10) and (-1 not in a_clamp_avoidance))):  
                                            print('无下避让条件，匀速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = 0.5
                                        else :
                                            print('无下避让条件，加速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = 1
                                elif ((state[0, 0] < state[a, 0]) and (driving_orgin == True)) or ((state[0, 0] > state[a, 0]) and (driving_orgin == False)): # 在本车后方
                                    print(a, '在本车后方')
                                    # 如果有前向碰撞风险，全减速通过
                                    if (( gap_ego_x < 0 
                                    and (( 0 < state[a, 2] - state[0, 2] < 10))) and (-1 not in a_clamp_avoidance)):
                                        print('无下避让条件，全减速通过')
                                        avoid_allow = -10
                                        deside_avoid_acc = -1.2 # -1.2代表全减速通过
                                    else:
                                        if gap_ego_x < 0 and (dis_gap_1 > self.s_1 or dis_gap_1 == -1) and (state[0, 2] > state[a, 2]):
                                            print('无下避让条件，全加速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = 1.2
                                        elif (np.absolute(v_1 - state[a, 2]) / self.a_bound * (state[a, 2]  + 1e-6)) < (gap_ego_x):
                                            print('无下避让条件，加速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = 1
                                        elif state[a, 2] - state[0, 2] >= 5 and (self.s_1 < dis_gap_1 or dis_gap_1 == -1):
                                            print('无上避让条件，匀速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = 0.5
                                        else:  
                                            print('无下避让条件，减速通过')
                                            avoid_allow = -10
                                            deside_avoid_acc = -1  
                            if  ((gap_ego_y < 0.3) 
                            and (0 < gap_ego_x < 1) 
                            and ((1 in a_lateral_avoidance) and (-1 in a_lateral_avoidance)) == False and lateral_avoid_allow): # 纵向距离小于0.2且横向距离小于0强制下偏
                                final_y = final_y - 1
                                avoid_allow = -1.1 # -1.1代表强制上避让
                                # deside_avoid_acc = 0.5 
                                print('强制下避让',a)
                                print('当前y值，中心线值',state[0, 1],final_y)  
            if (gap_ego_x < 70) and (avoid_allow != (1.1 and -1.1)): # 前后向防撞检测7米，侧向检测7米 
                # 前、后向避障算法
                if 0 >= (gap_ego_y): # 检测 i车辆与本车纵向距离，此处表明为前后跟车
                    # print(a, '与该车的纵向距离', gap_ego_y)
                    # print(a, '与本车有后向', dis_gap_2, gap_ego_x, (np.absolute(state[0, 2] - state[a, 2]) / (self.a_bound * state[a, 2] + 1e-6)))
                    if driving_orgin: # 正向驾驶场景
                        # 如果 a车在本车后方，且速度比本车大，变速距离大于与后车实际距离，或者大于前车实际距离,或者横向距离小于1，且与本车之间无其他车辆
                        if (((state[0, 0] < state[a, 0]) and (state[0, 2] < state[a, 2]) and dis_gap_2 == (gap_ego_x) and
                        ((((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[a, 2] + 1e-6)) > (gap_ego_x))
                        or ((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) > dis_gap_1))
                        or ((gap_ego_x) < 1)))
                        and (a_lateral_avoidance_after == False or (np.absolute(state[0, 0] - state[a, 0]) < a_lateral_avoidance_after)) 
                        and (np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) - ((gap_ego_x))) > 10): # 后向碰撞检测
                            a_lateral_avoidance_after = np.absolute(state[0, 0] - state[a, 0])
                            a_clamp_avoidance.append(-1)  # 前后碰撞参数记录
                            print(a, '与本车有后向碰撞风险', dis_gap_2, gap_ego_x, (np.absolute(state[0, 0] - state[a, 0]) - (state[0, 4] + state[a, 4]) / 2))
                            # 如果本车已在终点线车道，只做减速处理，提升终点通过率
                            if np.absolute(final_goal_y - state[0, 1]) > 0.6 or (gap_ego_x) < 5: 
                                # 终点在上方
                                if y_gap_true < -0.6 and final_y_acc_up == 1:
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = -2.1
                                    # deside_avoid_acc = 0.5 
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                # 终点在下方
                                elif y_gap_true > 0.6 and final_y_acc_down == 1:     
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = -2.2
                                    # deside_avoid_acc = 0.5 
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y) 
                                elif final_y_acc_up == 1:
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1 # 向目标车上方避让，避让中心线为该车纵坐标值+两车宽度一半+安全距离1
                                    avoid_allow = -2.1
                                    # deside_avoid_acc = 0.5 
                                    # print(final_y)
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                elif final_y_acc_down == 1:
                                    print(final_y)
                                    # final_y = final_y + np.absolute((np.absolute(state[a, 1] - state[0, 1]) - (state[a, 5] + state[0, 5])/2)) + 1
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = -2.2
                                    # deside_avoid_acc = 0.5 
                                    # print(final_y)
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                else:
                                    a_clamp_avoidance.append(-1)  # 前后碰撞参数记录
                                    # 此处加一个前后同时避障方法
                                    if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance) :
                                        if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                            deside_avoid_acc = 3 # 3 代表前后夹持场景，微调加速值
                                            print('后向车较近，执行加速') # 考虑加一个匀速空间，以平滑加速度曲线
                                        else:
                                            deside_avoid_acc = -3 # -3 代表前后夹持场景，微调减速值
                                            print('前向车较近，执行减速')
                                    else:
                                        deside_avoid_acc = 1.1 # 不满足变道条件，执行加速，后向碰撞风险加速为1.1
                                        print('不满足变道条件，执行加速')
                                        avoid_allow = -20 # 存在后向碰撞风险
                            else:
                                # 此处加一个前后同时避障方法
                                if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance):
                                    if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                        deside_avoid_acc = 3 # 3 代表前后夹持场景，微调加速值
                                        print('后向车较近，执行加速') # 考虑加一个匀速空间，以平滑加速度曲线
                                    else:
                                        deside_avoid_acc = -3 # -3 代表前后夹持场景，微调减速值
                                        print('前向车较近，执行减速')
                                elif dis_gap_1 > self.s_1 + state[0, 4]:
                                    deside_avoid_acc = 1.1 # 不满足变道条件，执行加速，后向碰撞风险加速为1.1
                                    print('不满足变道条件，执行加速')
                                    avoid_allow = -20 # 存在后向碰撞风险
                        if (((state[0, 0] > state[a, 0]) and (((state[0, 2] > state[a, 2]) and (self.s_1 - 1 > dis_gap_1) and 
                        dis_gap_1 == (gap_ego_x) and 
                        ((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) > (gap_ego_x)
                        or ((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (fv_2 + 1e-6)) > dis_gap_2)))
                        or ((np.absolute(state[0, 0] - state[a, 0]) - (state[0, 4] + state[a, 4]) / 2) < 1))) 
                        and  (a_lateral_avoidance_fomer == False or (np.absolute(state[0, 0] - state[a, 0]) < a_lateral_avoidance_fomer))
                        and (np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) - ((gap_ego_x))) > 10): # 前向碰撞检测
                            a_lateral_avoidance_fomer = np.absolute(state[0, 0] - state[a, 0])
                            a_clamp_avoidance.append(1)  # 前后碰撞参数记录
                            print(a, '与本车有前向碰撞风险')
                            # 如果本车已在终点线车道，只做减速处理，提升终点通过率
                            if (np.absolute(y_gap_true) > 0.6 or (gap_ego_x) < 5): 
                                
                                # 终点在上方
                                if y_gap_true < -0.6 and final_y_acc_up == 1:
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = 2.1
                                    # deside_avoid_acc = 0.5 
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                # 终点在下方
                                elif y_gap_true > 0.6 and final_y_acc_down == 1:     
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = 2.2
                                    # deside_avoid_acc = 0.5 
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                elif final_y_acc_up == 1:
                                    print(final_y)
                                    # final_y = final_y + np.absolute((np.absolute(state[a, 1] - state[0, 1]) - (state[a, 5] + state[0, 5])/2)) + 1
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = 2.1
                                    # deside_avoid_acc = 0.5 
                                    # print(final_y)
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                elif final_y_acc_down == 1:
                                    # print(final_y)
                                    # final_y = final_y + np.absolute((np.absolute(state[a, 1] - state[0, 1]) - (state[a, 5] + state[0, 5])/2)) + 1
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = 2.2
                                    # deside_avoid_acc = 0.5 
                                    # print(final_y)
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y) 
                                else:
                                    # 此处加一个前后同时避障方法
                                    if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance) :
                                        v_1, fv_1, dis_gap_1, direction_1, a_car_num_1 = self.getInformFront(state, 1, lane_now_yaw) # 取出前车数据
                                        v_2, fv_2, dis_gap_2, direction_2, a_car_num_2 = self.getInformFront(state, -1, lane_now_yaw) # 取出后车数据
                                        if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                            deside_avoid_acc = 3 # 3 代表前后夹持场景，微调加速值
                                            print('后向车较近，执行加速')
                                        else:
                                            deside_avoid_acc = -3 # -3 代表前后夹持场景，微调减速值
                                            print('前向车较近，执行减速')
                                    else:
                                        deside_avoid_acc = -1.1 # 不满足变道条件，执行减速,前向碰撞风险减速为-1.1
                                        avoid_allow = 20 # 存在前向碰撞风险
                            else:
                                # 此处加一个前后同时避障方法
                                    if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance):
                                        v_1, fv_1, dis_gap_1, direction_1, a_car_num_1 = self.getInformFront(state, 1, lane_now_yaw) # 取出前车数据
                                        v_2, fv_2, dis_gap_2, direction_2, a_car_num_2 = self.getInformFront(state, -1, lane_now_yaw) # 取出后车数据
                                        if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                            deside_avoid_acc = 3 # 3 代表前后夹持场景，微调加速值
                                            print('后向车较近，执行加速')
                                        else:
                                            deside_avoid_acc = -3 # -3 代表前后夹持场景，微调减速值
                                            print('前向车较近，执行减速')
                                    else:
                                        deside_avoid_acc = -1.1 # 不满足变道条件，执行减速,前向碰撞风险减速为-1.1
                                        avoid_allow = 20 # 存在前向碰撞风险

                    else: # 反向驾驶场景
                        # 如果 a车在本车后方，且速度比本车大，变速距离大于与后车实际距离，或者大于前车实际距离,或者横向距离小于1
                        if (((state[0, 0] > state[a, 0]) and (state[0, 2] < state[a, 2]) and  dis_gap_2 == (gap_ego_x) and
                        ((((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[a, 2] + 1e-6)) > (gap_ego_x))
                        or ((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) > dis_gap_1))
                        or ((gap_ego_x) < 1)))
                        and (a_lateral_avoidance_after == False or (np.absolute(state[0, 0] - state[a, 0]) < a_lateral_avoidance_after))
                        and (np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) - ((gap_ego_x))) > 10): # 后向碰撞检测
                            a_lateral_avoidance_after = np.absolute(state[0, 0] - state[a, 0])
                            a_clamp_avoidance.append(-1)  # 前后碰撞参数记录
                            print(a, '与本车有后向碰撞风险')
                            # 如果本车已在终点线车道，只做减速处理，提升终点通过率
                            if np.absolute(y_gap_true) > 0.6 or (gap_ego_x) < 5: 
                                # 终点在上方
                                if y_gap_true < -0.6 and final_y_acc_up == 1 and state[0, 1] > state[a, 1]:
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = -2.1
                                    # deside_avoid_acc = 0.5
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                # 终点在下方
                                elif y_gap_true > 0.6 and final_y_acc_down == 1 and state[0, 1] < state[a, 1]:     
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = -2.2
                                    # deside_avoid_acc = 0.5
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y) 
                                elif final_y_acc_down == 1 and state[0, 1] > state[a, 1]:
                                    # final_y = final_y - np.absolute((np.absolute(state[a, 1] - state[0, 1]) - (state[a, 5] + state[0, 5])/2)) - 1
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = -2.2
                                    # deside_avoid_acc = 0.5
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                elif final_y_acc_up == 1 and state[0, 1] > state[a, 1]:
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = -2.1
                                    # deside_avoid_acc = 0.5
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                else:
                                    # 此处加一个前后同时避障方法
                                    if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance):
                                        if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                            deside_avoid_acc = 3
                                            print('后向车较近，执行加速')
                                        else:
                                            deside_avoid_acc = -3
                                            print('前向车较近，执行减速')
                                    else:
                                        deside_avoid_acc = 1.1 # 不满足变道条件，执行加速
                                        avoid_allow = -20 # 存在后向碰撞风险
                            else:
                                # 此处加一个前后同时避障方法
                                if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance):
                                    if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                        deside_avoid_acc = 3
                                        print('后向车较近，执行加速')
                                    else:
                                        deside_avoid_acc = -3
                                        print('前向车较近，执行减速')
                                else:
                                    deside_avoid_acc = 1.1 # 不满足变道条件，执行加速
                                    avoid_allow = -20 # 存在后向碰撞风险
                        if (((state[0, 0] < state[a, 0]) and (((state[0, 2] > state[a, 2]) and (self.s_1 - 1 > dis_gap_1) and 
                        dis_gap_1 == (gap_ego_x) and
                        ((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (state[0, 2] + 1e-6)) > (gap_ego_x)
                        or ((np.absolute(state[0, 2] - state[a, 2]) / self.a_bound * (fv_2 + 1e-6)) > dis_gap_2)))
                        or ((gap_ego_x) < 1))) 
                        and  (a_lateral_avoidance_fomer == False or (np.absolute(state[0, 0] - state[a, 0]) < a_lateral_avoidance_fomer))
                        and (np.absolute(state[0, 0] - observation['test_setting']['goal']['x'][1]) - ((gap_ego_x))) > 10): # 前向碰撞检测
                            # self.s_ = self.s0 + v * (np.absolute(v - fv) / self.a_bound) + state[0, 4]
                            a_lateral_avoidance_fomer = np.absolute(state[0, 0] - state[a, 0])
                            a_clamp_avoidance.append(1) # 前后碰撞参数记录
                            print(a, '与本车有前向碰撞风险')
                             # 如果本车已在终点线车道，只做减速处理，提升终点通过率
                            if np.absolute(final_goal_y - state[0, 1]) > 0.4 or (gap_ego_x) < 5:
                                # 终点在上方
                                if y_gap_true < -0.6 and final_y_acc_up == 1:
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = 2.1
                                    # deside_avoid_acc = 0.5
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                # 终点在下方
                                elif y_gap_true > 0.6 and final_y_acc_down == 1:     
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = 2.2
                                    # deside_avoid_acc = 0.5
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y) 
                                elif final_y_acc_down == 1:
                                    final_y = state[a, 1] - (state[a, 5] + state[0, 5])/2 - 1
                                    avoid_allow = 2.2
                                    # deside_avoid_acc = 0.5
                                    print('下避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                elif final_y_acc_up == 1:
                                    print(final_y)
                                    final_y = state[a, 1] + (state[a, 5] + state[0, 5])/2 + 1
                                    avoid_allow = 2.1
                                    # deside_avoid_acc = 0.5
                                    print('上避让',a)
                                    print('当前y值，中心线值',state[0, 1],final_y)
                                else:
                                    # 此处加一个前后同时避障方法
                                    if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance):
                                        if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                            deside_avoid_acc = 1
                                            print('后向车较近，执行加速')
                                        else:
                                            deside_avoid_acc = -1
                                            print('前向车较近，执行减速')
                                    else:
                                        deside_avoid_acc = -1.1 # 不满足变道条件，执行减速
                                        avoid_allow = 20 # 存在前向碰撞风险
                            else:
                                # 此处加一个前后同时避障方法
                                if  (1 in a_clamp_avoidance) and (-1 in a_clamp_avoidance):
                                    if dis_gap_1 > dis_gap_2: # 后车距离较近，执行加速避障
                                        deside_avoid_acc = 1
                                        print('后向车较近，执行加速')
                                    else:
                                        deside_avoid_acc = -1
                                        print('前向车较近，执行减速')
                                else:
                                    deside_avoid_acc = -1.1 # 不满足变道条件，执行减速
                                    avoid_allow = 20 # 存在前向碰撞风险

        return final_y, avoid_allow, deside_avoid_acc

    # 定义可行驶中心线列表方法
    def deside_lane_center_drving_list(self, driving_orgin, state, lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list):
        lane_center_dring_list = []
        lane_center_dring_yaw_list = []
        ego_x_now = state[0, 0]
        if driving_orgin:
            # 构建可行使中心线列表
            a = -1 # V1.4 修改索引办法
            for i in lane_center_x_max: 
                a += 1
                if ego_x_now <= (i + 10) and ego_x_now >= (lane_center_x_min[a] - 10 ):  # V1.5更新可行使中心线两端判断，原来只判断一端，导致本车会在无道路区域行驶
                    lane_center_dring_list.append(lane_center[a])
                    lane_center_dring_yaw_list.append(lane_center_yaw_list[a])
        else:
            # 构建可行使中心线列表
            a = -1
            for i in lane_center_x_min:
                a += 1
                if ego_x_now >= (i - 10) and ego_x_now <= (lane_center_x_max[a] + 10):
                # if ego_x_now >= i:  
                    lane_center_dring_list.append(lane_center[a])
                    lane_center_dring_yaw_list.append(lane_center_yaw_list[a])
        return lane_center_dring_list, lane_center_dring_yaw_list

    # 车道数量输出方法
    def lanes_num(self, lane_info_list):
        # 车道数量初始为0
        lanes_num = 0
        # 检测 list中元素数量 
        for i in lane_info_list:
            # print(lane_info[lanes_num])
            lanes_num += 1
        return int(lanes_num/4)
        # print(luwang_list[0])

    # 路网信息提取方法
    def lane_info(self, luwang_list, lane_num, state):
        # 建立中心线坐标空列表
        lane_center = []
        # 建立中心线x_min坐标列表
        lane_center_x_min = []
        # 建立中心线x_max坐标列表
        lane_center_x_max = []
        # 建立各中心线道路的道路角度列表
        lane_center_yaw_list = []
        # 取出本车现在 x值
        x_now = state[0, 0]
        # id 内数组索引，用于索引道路 id
        id_num = 0
        # . 的出现次数，用于取出第三个数字，该数字代表 id
        dot_num = 0
        # luwang_list 索引，用于索引 id列表
        id_index = 0
        # 取出道路的宽度
        lane_width = 0
        for a in range(0, lane_num):
            # id 列表中取出 id号
            for b in luwang_list[id_index]:
                id_num += 1
                if b == '.':
                    dot_num += 1
                    if dot_num == 2:
                        id_1 = luwang_list[id_index][id_num]
                        id_2 = luwang_list[id_index][id_num+1]
                        if id_1 == "-":
                            # print('lane_id为：', id_1, int(id_2))
                            # 中心线初始置为 false
                            luwang_list_center = False
                            # 道路中心线索引计数
                            c_index = -1
                            for c in luwang_list[id_index+3][:]:
                                # print(c[0], '每行x值')
                                c_index += 1
                                if np.absolute(c[0] - x_now) <= 1:
                                    # print('道路中心线坐标为：', luwang_list[id_index+3][c_index][1])
                                    luwang_list_center = luwang_list[id_index+3][c_index][1]
                                    lane_center.append(luwang_list_center) # 道路中心线 y值
                                    break
                            # 如果无符合中心线，则取首项值
                            if luwang_list_center is False:
                                lane_center.append(luwang_list[id_index+3][0][1])
                                # print('道路中心线坐标为：', luwang_list[id_index+3][0][1])
                            if luwang_list[id_index + 3][0][0] < luwang_list[id_index + 3][-1][0]:
                                lane_center_x_min.append(luwang_list[id_index + 3][0][0])
                                lane_center_x_max.append(luwang_list[id_index + 3][-1][0])
                            else:
                                lane_center_x_min.append(luwang_list[id_index + 3][-1][0])
                                lane_center_x_max.append(luwang_list[id_index + 3][0][0])
                            # 计算并存储当前道路角度到列表（取道路的边界值进行反三角函数计算）
                            if luwang_list[id_index + 3][0][0] < luwang_list[id_index + 3][-1][0]:
                                if c_index + 10 > len(luwang_list[id_index + 3][:]) - 1:
                                    c_index_next = len(luwang_list[id_index + 3][:]) - 1
                                else:
                                    c_index_next = c_index + 10
                                x_min = luwang_list[id_index + 3][c_index][0]
                                x_max = luwang_list[id_index + 3][c_index_next][0]
                                y_min = luwang_list[id_index + 3][c_index][1]
                                y_max = luwang_list[id_index + 3][c_index_next][1]
                            else:
                                if c_index - 10 < 0:
                                    c_index_next = 0
                                else:
                                    c_index_next = c_index - 10
                                x_min = luwang_list[id_index + 3][c_index][0]
                                x_max = luwang_list[id_index + 3][c_index_next][0]
                                y_min = luwang_list[id_index + 3][c_index][1]
                                y_max = luwang_list[id_index + 3][c_index_next][1]
                            lane_center_yaw = np.arctan((y_max - y_min) / (x_max - x_min + 1e-6))
                            lane_center_yaw_list.append(lane_center_yaw)
                            lane_width = np.abs(luwang_list[id_index+1][0][1] - luwang_list[id_index+2][0][1])
                            # print(lane_center_yaw_list, 'lane_center_yaw_list')
                            # print(lane_center_yaw, 'lane_center_yaw')
                            id_index += 4
                            dot_num = 0
                            id_num = 0
                            break
                        else:
                            # print('lane_id为：', id_1)
                            luwang_list_center = False
                            c_index = -1
                            for c in luwang_list[id_index+3][:]:
                                # print(c[0], '每行x值')
                                c_index += 1
                                if np.absolute(c[0] - x_now) <= 1:
                                    # print('道路中心线坐标为：', luwang_list[id_index+3][c_index][1])
                                    luwang_list_center = luwang_list[id_index+3][c_index][1]
                                    lane_center.append(luwang_list_center) # 道路中心线 y值
                                    break
                            # 如果无符合中心线，则取首项值
                            if luwang_list_center is False:
                                lane_center.append(luwang_list[id_index+3][0][1])
                                # print('道路中心线坐标为：', luwang_list[id_index+3][0][1])
                            if luwang_list[id_index + 3][0][0] < luwang_list[id_index + 3][-1][0]:
                                lane_center_x_min.append(luwang_list[id_index + 3][0][0])
                                lane_center_x_max.append(luwang_list[id_index + 3][-1][0])
                            else:
                                lane_center_x_min.append(luwang_list[id_index + 3][-1][0])
                                lane_center_x_max.append(luwang_list[id_index + 3][0][0])
                            # 计算并存储当前道路角度到列表（取道路的边界值进行反三角函数计算）
                            if luwang_list[id_index + 3][0][0] < luwang_list[id_index + 3][-1][0]:
                                if c_index + 10 > len(luwang_list[id_index + 3][:]) - 1:
                                    c_index_next = len(luwang_list[id_index + 3][:]) - 1
                                else:
                                    c_index_next = c_index + 10
                                x_min = luwang_list[id_index + 3][c_index][0]
                                x_max = luwang_list[id_index + 3][c_index_next][0]
                                y_min = luwang_list[id_index + 3][c_index][1]
                                y_max = luwang_list[id_index + 3][c_index_next][1]
                            else:
                                if c_index - 10 < 0:
                                    c_index_next = 0
                                else:
                                    c_index_next = c_index - 10
                                x_min = luwang_list[id_index + 3][c_index][0]
                                x_max = luwang_list[id_index + 3][c_index_next][0]
                                y_min = luwang_list[id_index + 3][c_index][1]
                                y_max = luwang_list[id_index + 3][c_index_next][1]
                            lane_center_yaw = np.arctan((y_max - y_min) / (x_max - x_min + 1e-6))
                            lane_center_yaw_list.append(lane_center_yaw)
                            lane_width = np.abs(luwang_list[id_index+1][0][1] - luwang_list[id_index+2][0][1])
                            # print(lane_center_yaw_list, 'lane_center_yaw_list')
                            # print(lane_center_yaw, 'lane_center_yaw')
                            id_index += 4
                            dot_num = 0
                            id_num = 0
                            break
        return lane_center, lane_center_x_max, lane_center_x_min, lane_center_yaw_list, lane_width

    # 判定当前车所属中心线，以及该中心线剩余可行驶距离 更改为判定本车y值0.2米范围内延申的20米距离是否与一可行使中心线重合
    def deside_lane_center_drving_distance(self, state, final_i_now, lane_center_dring_list, lane_center, lane_center_x_max, lane_center_x_min, driving_orgin):
        lane_center_dring_distance = 0
        ego_x_now = state[0, 0]
        ego_y_now = final_i_now
        # 寻找并输出可行使距离
        a = -1 # 索引值
        if driving_orgin: # 正向驾驶场景
            for i in lane_center_x_max: # 遍历中心线边界值列表
                a += 1
                # 检测本车距离道路终点距离，以及检测该道路是否有延申路段，检测到道路延申，则更新剩余距离
                if((ego_x_now <= (i + lane_center_dring_distance + 10)) and (ego_x_now >= (lane_center_x_min[a] - lane_center_dring_distance - 10 )) 
                and (np.absolute(ego_y_now - lane_center[a]) <= 2)):  # 判断本车所在道路
                    lane_center_dring_distance =  ego_x_now - lane_center_x_min[a] - state[0, 4]/2

        else:
            for i in lane_center_x_min: 
                a += 1
                if ((ego_x_now >= (i - lane_center_dring_distance - 10)) and (ego_x_now <= (lane_center_x_max[a] + lane_center_dring_distance + 10 )) and
                 (np.absolute(ego_y_now - lane_center[a]) <= 2)):  # 判断本车所在道路
                    lane_center_dring_distance = lane_center_x_max[a] - ego_x_now - state[0, 4]/2
        lane_center_dring_distance += 100
        # 考虑增加道路衔接判定，避免拼接道路对剩余可行使距离的影响
        return lane_center_dring_distance
                    
    # 定义一个考虑道路倾斜情况的其他车辆与本车横纵向距离计算方法
    def deside_ego_gap(self, state, lane_now_yaw, a):
        gap_ego_y = np.cos(lane_now_yaw) * ((np.abs(state[0, 1] - state[a, 1])) - np.tan(lane_now_yaw) * (np.abs(state[0, 0] - state[a, 0]))) - ((state[0, 5] + state[a, 5])/2)
        gap_ego_x = np.absolute(state[a, 0] - state[0, 0]) / (np.cos(lane_now_yaw) + 1e-6) - (state[0, 4] + state[a, 4])/2
        return gap_ego_x, gap_ego_y
    
    # 定义一个考虑道路倾斜情况的其他车辆与本车上下关系的计算方法
    def deside_ego_location(self, state, lane_now_yaw, a):
        b = state[0, 1] - np.tan(lane_now_yaw) * state[0, 0]
        ego_other_y = np.tan(lane_now_yaw) * state[a, 0] + b
        return ego_other_y
    
    # 定义一个考虑道路倾斜情况的是否在终点区域的计算方法
    def deside_ego_final(self, state, lane_now_yaw, final_y, final_space, observation):
        yaw_up = False
        yaw_down = False
        yaw_hold = False
        b = state[0, 1] - np.tan(lane_now_yaw) * state[0, 0]
        x_final = observation['test_setting']['goal']['x'][1]
        final_dy = final_space / (np.cos(lane_now_yaw) + 1e-6)      # 本车与终点的纵向差值
        y_final = np.tan(lane_now_yaw) * x_final + b    # 本车在终点x的对应y值
        # if y_final + final_dy < final_y:
        if final_y - final_dy > y_final:
            yaw_up = True # 终点在下方
        # elif y_final - final_dy > final_y:
        elif final_y + final_dy < y_final:
            yaw_down = True # 终点在上方
        else:
            yaw_hold = True # 在目标区域内
        # 加一个算本车与中心线纵向距离的方法
        y_gap = y_final - final_y # 本车值 - 中心线值
        y_gap_true = np.cos(lane_now_yaw) * y_gap
        return yaw_up, yaw_down, yaw_hold, y_gap_true
    # 定义一个计算本车与当前道路中心线纵向距离的方法
    def deside_ego_now_gap(self, state, lane_now_yaw, lane_y_now, ego_num):
        y_gap = state[int(ego_num), 1] - lane_y_now
        y_gap_true = np.cos(lane_now_yaw) * y_gap
        return y_gap_true